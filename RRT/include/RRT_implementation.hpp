#include <iostream>
#include "./RRT.hpp"
#include "./utils.hpp"

namespace rrt
{
	template <class T>
	void RRT<T>::setEndPoints(Utils::Point<T> start, Utils::Point<T> end)
	{
		startPoint=start;
		endPoint=end;
	}

	template <class T>
	void RRT<T>::setHalfDimensions(double a, double b)
	{
		halfDimensionX=a;
		halfDimensionY=b;
	}

	template <class T>
	void RRT<T>::setOrigin(Utils::Point<T> pt)
	{
		origin=pt;
	}

	template <class T>
	std::deque<Utils::Point<T> > RRT<T>::getPointsOnPath()
	{
		return pathPoints;
	}

	template <class T>
	void RRT<T>::setStepLength(double value)
	{
		stepLength=value;
	}

	template <class T>
	void RRT<T>::setMaxIterations(int value)
	{
		maxIterations=value;
	}

	template <class T>
	bool RRT<T>:: plan()
	{
		int count=0;
		int check=0;
		tree.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (startPoint,startPoint));
		while( check < maxIterations)
		{
			//std::cout<<"In Planning Loop"<<std::endl;
			Utils::Point<T> next;
			int arr[]={1};
			std::pair  <Utils::Point <T>,Utils::Point <T> >  mid = treeComplete(arr);
			if(mid.first!=mid.second)
			{
				std::cout<<"Tree complete!!"<<std::endl;
				int start_time=clock();
				growTree(endPoint);
				int end_time=clock();
				std::cout<<"Time to generate path = "<<end_time-start_time<<std::endl;
				generatePath(startPoint,endPoint);
				return true;
			}
			else if(count%biasParameter==0)
			{
				//std::cout<<"Adding Next Biased Point to tree!!"<<std::endl;
				count=0;
				do{
					next= generateBiasedPoint(1);
				}while(checkPoint(next)!=true);
				//std::cout<<" : "<<next.x<<","<<next.y<<std::endl;
			}
			else
			{
				//std::cout<<"Adding next point to tree!!"<<std::endl;
				do{
					next = generatePoint();
				}while(checkPoint(next)!=true);
				//std::cout<<" : "<<next.x<<","<<next.y<<std::endl;
			}
			//std::cout<<" Growing Tree next : "<<next.x<<","<<next.y<<std::endl;
			growTree(next);
			count++;
			check++;
			//std::cout<<"check= "<<check<<", count= "<<count<<std::endl;
		}
		std::cout<<"Path Not Found"<<std::endl;
		return false;
	}

	template <class T>
	void RRT<T>::growTree(Utils::Point<T> next)
	{
		//growing the tree by adding node
		//std::cout<<"finding parent in tree of size = "<<tree.size()<<std::endl;
		Utils::Point<T> parent;
		parent= findClosestNode(next).first;
		//std::cout<<"current : "<<next.x<<","<<next.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
		tree.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (next,parent));
		//std::cout<<"Tree grown"<<std::endl;
		//add pruning code to keep size of tree under control
	}

	template <class T>
	void RRT<T>::setCheckPointFunction(bool (*f)(Utils::Point<T>))
	{
		userCheck=f;
	}

	template <class T>
	std::pair<Utils::Point<T>, int>  RRT<T>::findClosestNode(Utils::Point<T> cur)
	{
		double min_dist=INT_MAX;
		Utils::Point<T> closest;
		for(int i=0;i<tree.size();i++)
		{
			////std::cout<<"Itering the tree to find closest Node"<<std::endl;
			if(dist(tree[i].first,cur) < min_dist )
			{
				min_dist=dist(tree[i].first,cur);
				closest=tree[i].first;
			}
		}
		return std::make_pair(closest,1);
	}

	template <class T>
	Utils::Point<T> RRT<T>::generatePoint()
	{
		Utils::Point<T> temp;
		int signX,signY;

		if(rand()%2==0)
			signX=-1;
		else
			signX=1;
		if(rand()%2==0)
			signY=-1;
		else
			signY=1;

		temp.x = origin.x + signX* ( rand() % (int)halfDimensionX );
		temp.y = origin.y + signY* ( rand() % (int)halfDimensionY );

		Utils::Point<T> closest=findClosestNode(temp).first;
		double theta = atan2(temp.y-closest.y,temp.x-closest.x);

		Utils::Point<T> next;
		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		//std::cout<<"Point Generated = "<<temp.x<<","<<temp.y<<std::endl;
		return next;
	}

	template <class T>
	Utils::Point<T> RRT<T>::generateBiasedPoint(int which)
	{
		Utils::Point<T> closest=findClosestNode(endPoint).first;
		double theta = atan2(endPoint.y-closest.y,endPoint.x-closest.x);

		Utils::Point<T> next;
		next.x=closest.x+stepLength*cos(theta);
		next.y=closest.y+stepLength*sin(theta);
		//std::cout<<"Biased Point Generated = "<<next.x<<","<<next.y<<std::endl;
		return next;
	}

	template <class T>
	bool RRT<T>::checkPoint(Utils::Point<T> next)
	{
		return userCheck(next);
	}

	template <class T>
	std::pair <Utils::Point <T>,Utils::Point <T> > RRT<T>::treeComplete(int arr[1])
	{
		//std::cout<<"Checking if tree is complete? "<<std::endl;
		//std::cout<<"tree size = "<<tree.size()<<std::endl;
		int min_dist=INT_MAX;
		for(int i=0;i<tree.size();i++)
		{
			double dis= dist(tree[i].first,endPoint);
			if(dis < min_dist)
				min_dist=dis;
			if(dis < stepLength )
			{
				return std::make_pair(tree[i].first,endPoint);
			}
		}
		//std::cout<<"Min Distance In this iteration = "<<min_dist<<std::endl;
		return std::make_pair(endPoint,endPoint);
	}

	template <class T>
	void RRT<T>::setBiasParameter(unsigned int param)
	{
		biasParameter=param;
	}

	template <class T>
	void RRT<T>::generatePath(Utils::Point<T> first,Utils::Point<T> last)
	{
		Utils::Point<T> cur=last;
		pathPoints.push_back(cur);
		while(cur!=first || cur!=startPoint)
		{
			Utils::Point<T> parent= getParent(cur);
			//std::cout<<"current : "<<cur.x<<","<<cur.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
			pathPoints.push_back(parent);
			cur=parent;
		}
	}

	template <class T>
	Utils::Point<T> RRT<T>::getParent(Utils::Point<T> cur)
	{
		for(int i=0;i<tree.size();i++)
		{
			if(tree[i].first==cur)
				return tree[i].second;
		}
	}

	template <class T>
	double RRT<T>::dist(Utils::Point<T> a,Utils::Point<T> b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
}
