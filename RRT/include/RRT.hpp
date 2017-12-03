#ifndef _RRT_
#define _RRT_

#include <iostream>
#include <bits/stdc++.h>
#include "utils.hpp"

namespace rrt
{
	template <class T>
	class RRT
	{
	private:
		double halfDimensionX;
		double halfDimensionY;
		Utils::Point<T> origin;
		Utils::Point<T> startPoint;
		Utils::Point<T> endPoint;
		double stepLength;
		std::deque<Utils::Point<T> > pathPoints;
		int maxIterations;
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree; 
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree1; 
		std::vector< std::pair< Utils::Point<T>, Utils::Point<T> > > tree2; 
		unsigned int biasParameter;
		
	public:
		
		RRT(){};
		RRT(Utils::Point<T> start,Utils::Point<T> end)
		{
			//srand(time(NULL));			
			startPoint=start;
			endPoint=end;
		}
		
		virtual bool plan();
		virtual std::deque<Utils::Point<T> > getPointsOnPath();

		virtual void setEndPoints(Utils::Point<T> start, Utils::Point<T> end);
		virtual void setCheckPointFunction(bool (*f)(Utils::Point<T>));
		virtual void setStepLength(double value);
		virtual void setOrigin(Utils::Point<T> origin);
		virtual void setHalfDimensions(double x,double y);
		virtual void setBiasParameter(unsigned int);
		virtual void setMaxIterations(int);
		//TODO : To be implemented in the derived classes
		// virtual void interpolate();
		// virtual void fitVelocityProfile();
		// virtual void pruneTree();

	private:
		bool (*userCheck)(Utils::Point<T>);
		bool checkPoint(Utils::Point<T> pt);
		Utils::Point<T> generatePoint();
		Utils::Point<T> generateBiasedPoint(int);
		void growTree(Utils::Point<T>);
		std::pair<Utils::Point<T>, int>  findClosestNode(Utils::Point<T>);
		Utils::Point<T> getParent(Utils::Point<T>);
		std::pair <Utils::Point <T>,Utils::Point <T> > treeComplete(int*);
		void generatePath(Utils::Point<T> first,Utils::Point<T> last);
		double dist(Utils::Point<T> a,Utils::Point<T> b);
	};
}

//************* ToDo ************* 
// Implement derived classes for variants of RRT
// Optimize the generate path and other 
// Tweak with the parameters to check their effects on runtime and path 
// Implement Pruning function to keep a check on size of tree


#endif