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
		int which=1;
		int check=0;
		tree1.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (startPoint,startPoint));
		tree2.push_back(std::pair< Utils::Point<T>, Utils::Point<T> > (endPoint,endPoint));
		while( check < maxIterations)
		{
			//std::cout<<"In Planning Loop"<<std::endl;
			Utils::Point<T> next;
			int flag[]={0};
			std::pair <Utils::Point<T>, Utils::Point<T> > mid = treeComplete(flag);
			if(flag[0])
			{
				std::cout<<"Tree complete!!"<<std::endl;
				// growTree(endPoint);	//need to look into
				int start_time=clock();
				generatePath(mid.first,mid.second);
				int end_time=clock();
				std::cout<<"Time to generate path = "<<end_time-start_time<<std::endl;
				return true;
			}
			else if(count%biasParameter==0)
			{
				//std::cout<<"Adding Next Biased Point to tree!!"<<std::endl;
				count=0;
				do{
					next= generateBiasedPoint(which);
					which*=-1;
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
		std::pair < Utils::Point<T> , int > both= findClosestNode(next);
		Utils::Point<T> parent=both.first;
		int which = both.second;
		//std::cout<<"current : "<<next.x<<","<<next.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
		if (which)	
			tree2.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (next,parent));
		else
			tree1.push_back( std::pair< Utils::Point<T>, Utils::Point<T> > (next,parent));

		//std::cout<<"Tree grown"<<std::endl;
		//add pruning code to keep size of tree under control
	}

	template <class T>
	void RRT<T>::setCheckPointFunction(bool (*f)(Utils::Point<T>))
	{
		userCheck=f;
	}

	template <class T>
	std::pair<Utils::Point<T>, int> RRT<T>::findClosestNode(Utils::Point<T> cur)
	{
		int which=0;
		double min_dist=INT_MAX;
		Utils::Point<T> closest;
		for(int i=0;i<tree1.size();i++)
		{
			////std::cout<<"Itering the tree to find closest Node"<<std::endl;
			if(dist(tree1[i].first,cur) < min_dist )
			{
				min_dist=dist(tree1[i].first,cur);
				closest=tree1[i].first;
			}
		}
		for(int i=0;i<tree2.size();i++)
		{
			////std::cout<<"Itering the tree to find closest Node"<<std::endl;
			if(dist(tree2[i].first,cur) < min_dist )
			{
				min_dist=dist(tree2[i].first,cur);
				closest=tree2[i].first;
				which=1;

			}
		}
		return std::make_pair(closest, which);
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
	Utils::Point<T> RRT<T>::generateBiasedPoint(int where)
	{
		Utils::Point<T> next;
		if (where==1)
		{
			Utils::Point<T> closest=findClosestNode(startPoint).first;
			double theta = atan2(endPoint.y-closest.y,endPoint.x-closest.x);

			next.x=closest.x+stepLength*cos(theta);
			next.y=closest.y+stepLength*sin(theta);
		}
		else
		{
			Utils::Point<T> closest=findClosestNode(endPoint).first;
			double theta = atan2(endPoint.y-closest.y,endPoint.x-closest.x);

			next.x=closest.x+stepLength*cos(theta);
			next.y=closest.y+stepLength*sin(theta);
		}
		//std::cout<<"Biased Point Generated = "<<next.x<<","<<next.y<<std::endl;
		return next;
	}

	template <class T>
	bool RRT<T>::checkPoint(Utils::Point<T> next)
	{
		return userCheck(next);
	}

	template <class T>
	std::pair <Utils::Point <T>,Utils::Point <T> > RRT<T>::treeComplete(int flag[1])
	{
		//std::cout<<"Checking if tree is complete? "<<std::endl;
		//std::cout<<"tree size = "<<tree.size()<<std::endl;
		int min_dist=INT_MAX;	
		Utils::Point <T> first, second;
		for(int i=0;i<tree1.size();i++)
		{
			for (int j=0;j<tree2.size();j++)
			{
				double dis= dist(tree1[i].first,tree2[j].first);
				if(dis < min_dist)
					min_dist=dis;
				if(dis < stepLength )
				{
					flag[0]=1;
					first=tree1[i].first;
					second=tree2[j].first;
					// std::cout<<"first: "<<first.x<<","<<first.y<<" sec: "<<second.x<<", "<<second.y<<"\n";
					return std::make_pair(first,second);
				}
				
			}
		}
		//std::cout<<"Min Distance In this iteration = "<<min_dist<<std::endl;
		return std::make_pair(first,second);
	}

	template <class T>
	void RRT<T>::setBiasParameter(unsigned int param)
	{
		biasParameter=param;
	}

	template <class T>
	void RRT<T>::generatePath(Utils::Point<T> first,Utils::Point<T> last)
	{
		// for (int i=0;i<tree2.size();i++)
		// {
		// 	Utils::Point<T> temp = tree2[i].first;
		// 	tree2[i].first = tree2[i].second;
		// 	tree2[i].second = temp;
		// } // Assuming generate point is called only once in the entire run of the program
		// tree1.insert(tree1.end(),tree2.begin(),tree2.end())

		Utils::Point<T> cur=last;
		pathPoints.push_back(cur);
		int count=0;
		while(cur!=endPoint)
		{
			Utils::Point<T> parent= getParent(cur);
			//std::cout<<"current : "<<cur.x<<","<<cur.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
			pathPoints.push_back(parent);
			cur=parent;
			// std::cout<<cur.x<<" "<<cur.y<<"\n";
			// std::cout<<count++<<std::endl;
		}

		cur=first;
		pathPoints.push_front(cur);
		count=0;
		while(cur!=startPoint)
		{
			Utils::Point<T> parent= getParent(cur);
			//std::cout<<"current : "<<cur.x<<","<<cur.y<<"| parent : "<<parent.x<<","<<parent.y<<std::endl;
			pathPoints.push_front(parent);
			cur=parent;
			// std::cout<<cur.x<<" "<<cur.y<<"\n";
			// std::cout<<count--<<std::endl;
		}
	}

	template <class T>
	Utils::Point<T> RRT<T>::getParent(Utils::Point<T> cur)
	{
		for(int i=0;i<tree1.size();i++)
		{
			if(tree1[i].first==cur)
				return tree1[i].second;
		}
		for(int i=0;i<tree2.size();i++)
		{
			if(tree2[i].first==cur)
				return tree2[i].second;
		}
	}

	template <class T>
	double RRT<T>::dist(Utils::Point<T> a,Utils::Point<T> b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
}

