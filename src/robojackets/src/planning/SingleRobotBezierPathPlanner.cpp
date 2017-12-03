#include "planning/SingleRobotBezierPathPlanner.hpp"
// #include "EscapeObstaclesPathPlanner.hpp"
#include <Constants.hpp>
#include <Utils.hpp>
// #include <protobuf/LogFrame.pb.h>
#include "TrapezoidalMotion.hpp"
#include "planning/Util.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>

#include <ros/console.h>

using namespace std;
using namespace Eigen;

namespace Planning {

  
InterpolatedPath* SingleRobotBezierPathPlanner::optimize(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    unsigned int start = 0;

    if (path.empty()) {
        delete &path;
        return nullptr;
    }

    vector<InterpolatedPath::Entry>& pts = path.waypoints;

    // The set of obstacles the starting point was inside of
    const auto startHitSet = obstacles->hitSet(pts[start].pos());
    int span = 2;
    while (span < pts.size()) {
        bool changed = false;
        for (int i = 0; i + span < pts.size(); i++) {
            bool transitionValid = true;
            const auto newHitSet = obstacles->hitSet(
                Geometry2d::Segment(pts[i].pos(), pts[i + span].pos()));
            if (!newHitSet.empty()) {
                for (std::shared_ptr<Geometry2d::Shape> hit : newHitSet) {
                    if (startHitSet.find(hit) == startHitSet.end()) {
                        transitionValid = false;
                        break;
                    }
                }
            }

            if (transitionValid) {
                for (int x = 1; x < span; x++) {
                    pts.erase(pts.begin() + i + 1);
                }
                changed = true;
            }
        }

        if (!changed) span++;
    }
    // Done with the path
    return cubicBezier(path, obstacles, motionConstraints, vi, vf);
}

// already defined in RRTPlanner, we just declare here...
float getTime(InterpolatedPath& path, int index,
              const MotionConstraints& motionConstraints, float startSpeed,
              float endSpeed);/* {
    return Trapezoidal::getTime(
        path.length(0, index), path.length(), motionConstraints.maxSpeed,
        motionConstraints.maxAcceleration, startSpeed, endSpeed);
}
*/
// TODO: Use targeted end velocity
InterpolatedPath* SingleRobotBezierPathPlanner::cubicBezier(
    InterpolatedPath& path, const Geometry2d::ShapeSet* obstacles,
    const MotionConstraints& motionConstraints, Geometry2d::Point vi,
    Geometry2d::Point vf) {
    int length = path.waypoints.size();
    int curvesNum = length - 1;
    if (curvesNum <= 0) {
        delete &path;
        return nullptr;
    }

    // TODO: Get the actual values
    vector<double> pointsX(length);
    vector<double> pointsY(length);
    vector<double> ks(length - 1);
    vector<double> ks2(length - 1);

    for (int i = 0; i < length; i++) {
        pointsX[i] = path.waypoints[i].pos().x;
        pointsY[i] = path.waypoints[i].pos().y;
    }
    float startSpeed = vi.mag();

    // This is pretty hacky;
    Geometry2d::Point startDirection =
        (path.waypoints[1].pos() - path.waypoints[0].pos()).normalized();
    if (startSpeed < 0.3) {
        startSpeed = 0.3;
        vi = startDirection * startSpeed;
    } else {
        vi = vi.mag() * (startDirection + vi.normalized()) / 2.0 * 0.8;
    }

    const float endSpeed = vf.mag();

    for (int i = 0; i < curvesNum; i++) {
        ks[i] = 1.0 /
                (getTime(path, i + 1, motionConstraints, startSpeed, endSpeed) -
                 getTime(path, i, motionConstraints, startSpeed, endSpeed));
        ks2[i] = ks[i] * ks[i];
        if (std::isnan(ks[i])) {
            delete &path;
            return nullptr;
        }
    }

    VectorXd solutionX = cubicBezierCalc(vi.x, vf.x, pointsX, ks, ks2);
    VectorXd solutionY = cubicBezierCalc(vi.y, vf.y, pointsY, ks, ks2);

    Geometry2d::Point p0, p1, p2, p3;
    vector<InterpolatedPath::Entry> pts;
    const int interpolations = 10;
    double time = 0;

    for (int i = 0; i < curvesNum; i++) {
        p0 = path.waypoints[i].pos();
        p3 = path.waypoints[i + 1].pos();
        p1 = Geometry2d::Point(solutionX(i * 2), solutionY(i * 2));
        p2 = Geometry2d::Point(solutionX(i * 2 + 1), solutionY(i * 2 + 1));

        for (int j = 0; j < interpolations; j++) {
            double k = ks[i];
            float t = (((float)j / (float)(interpolations)));
            Geometry2d::Point pos =
                pow(1.0 - t, 3) * p0 + 3.0 * pow(1.0 - t, 2) * t * p1 +
                3 * (1.0 - t) * pow(t, 2) * p2 + pow(t, 3) * p3;
            t = ((float)j / (float)(interpolations)) / k;
            // 3 k (-(A (-1 + k t)^2) + k t (2 C - 3 C k t + D k t) + B (1 - 4 k
            // t + 3 k^2 t^2))
            Geometry2d::Point vel =
                3 * k * (-(p0 * pow(-1 + k * t, 2)) +
                         k * t * (2 * p2 - 3 * p2 * k * t + p3 * k * t) +
                         p1 * (1 - 4 * k * t + 3 * pow(k, 2) * pow(t, 2)));
            pts.emplace_back(MotionInstant(pos, vel), time + t);
        }
        time += 1.0 / ks[i];
    }
    pts.emplace_back(MotionInstant(path.waypoints[length - 1].pos(), vf), time);
    path.waypoints = pts;
    return &path;
}

VectorXd SingleRobotBezierPathPlanner::cubicBezierCalc(double vi, double vf,
                                     vector<double>& points, vector<double>& ks,
                                     vector<double>& ks2) {
    int curvesNum = points.size() - 1;

    if (curvesNum == 1) {
        VectorXd vector(2);
        vector[0] = vi / (3.0 * ks[0]) + points[0];
        vector[1] = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);
        return vector;
    } else {
        int matrixSize = curvesNum * 2;
        MatrixXd equations = MatrixXd::Zero(matrixSize, matrixSize);
        VectorXd answer(matrixSize);
        equations(0, 0) = 1;
        answer(0) = vi / (3.0 * ks[0]) + points[0];
        equations(1, matrixSize - 1) = 1;
        answer(1) = points[curvesNum] - vf / (3 * ks[curvesNum - 1]);

        int i = 2;
        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2 + 1) = ks[n];
            equations(i, n * 2 + 2) = ks[n + 1];
            answer(i) = (ks[n] + ks[n + 1]) * points[n + 1];
            i++;
        }

        for (int n = 0; n < curvesNum - 1; n++) {
            equations(i, n * 2) = ks2[n];
            equations(i, n * 2 + 1) = -2 * ks2[n];
            equations(i, n * 2 + 2) = 2 * ks2[n + 1];
            equations(i, n * 2 + 3) = -ks2[n + 1];
            answer(i) = points[n + 1] * (ks2[n + 1] - ks2[n]);
            i++;
        }

        ColPivHouseholderQR<MatrixXd> solver(equations);
        VectorXd solution = solver.solve(answer);
        return solution;
    }
}
}