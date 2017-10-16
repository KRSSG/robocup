#include <iostream>
#include "../include/RRT_implementation.hpp"

using namespace std;
using namespace rrt;

bool check(Utils::Point<int> cur)
{
	if(abs(cur.x)<2000 && abs(cur.y)<2000)
		return true;
	return false;
}

int main()
{
	srand(time(NULL));
	Utils::Point<int> start,finish,origin;
	start.x=-10;
	start.y=10;
	finish.x=1000;
	finish.y=730;
	origin.x=0;
	origin.y=0;

	RRT<int> test;
	test.setEndPoints(start,finish);
	test.setCheckPointFunction(*(check));
	test.setStepLength(200);
	test.setHalfDimensions(2000.0,2000.0);
	test.setBiasParameter(100);
	test.setOrigin(origin);
	test.setMaxIterations(10000);
	test.plan();
	cout<<"#################################################"<<endl;
	vector<Utils::Point<int> > path=test.getPointsOnPath();
	for(int i=0;i<path.size();i++)
		cout<<path[i].x<<","<<path[i].y<<endl;
}
