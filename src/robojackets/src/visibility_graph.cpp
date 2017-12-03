//Final edit

#include "iostream"
#include "fstream"
#include <deque>
#include <cmath>
#include <climits>
#include <queue>
using namespace std;

const int botRadius = 5;
//const int R = 1.414*botRadius;
const int R = 5;
const int nbot = 3;

ofstream file;

typedef struct botcoordinate
{
	int x;
	int y;
	int botid;
	int edgewt;
	deque<botcoordinate> edgev;
}botcoordinate;

struct nodeDistance
{
    int node;
    unsigned int distance;
};

deque<botcoordinate> graph;
deque<botcoordinate> list;
deque<botcoordinate> vertex;

class CompareDist
{
    public:
        bool operator()(nodeDistance& n1, nodeDistance& n2)
        {
           if (n1.distance < n2.distance) 
                return true;
           else
                return false;
        }
};

void dijkstra(int s,int e) 
{
	//s is the starting node     
    bool *visited = new bool [graph.size()];
    unsigned int *dist = new unsigned int [graph.size()];
    int *previous=new int [graph.size()];

    // initialize the dist of each node as infinity and visited status as false
    for (int i = 0; i <graph.size(); ++i) 
    {
        dist[i] = INT_MAX;
        visited[i] = false;
    }

    // the distance of the source to itself is 0
    dist[s] = 0;

    // instantiate a priority queue with the structure and comparison criteria
    // as defined above
    priority_queue< nodeDistance, vector< nodeDistance >, CompareDist> pq;

    // Create the first node as the source and put it into the queue
    nodeDistance first = {s,0};
    pq.push(first);

    // While queue is not empty, pick the topmost node
    // using it's neighbors update the distance of each node that can be reached
    // and insert that node in the priority queue
    while(!pq.empty())
    {
        nodeDistance temp = pq.top();
        pq.pop();
        int node= temp.node;
           if(visited[node])
                      continue;
           visited[node]=true;
        for(int i=0;i < graph[node].edgev.size();i++)
        {
        	int pos=0;
        	int j=0;
        	while(j<graph.size())
        	{
        		if(vertex[j].x==(graph[node].edgev)[i].x && vertex[j].y==(graph[node].edgev)[i].y)
        		{
        			pos=j;
        			break;
        		}
        		j++;
        	}
            if(!visited[pos])
            {
                // Update the distance if it is smaller than the current distance
                int t= dist[node]  +  (graph[node].edgev)[i].edgewt;
                if(dist[pos]> t)
                {   dist[pos]=t;
                	previous[pos]=node;
                    nodeDistance newNode;                   
                    newNode.node=i;
                    newNode.distance=dist[pos];
                    pq.push(newNode);               
                }
            }
        }       
    }
    deque<botcoordinate> path;
    int position=e;
    botcoordinate temp;
    temp.x=graph[position].x;
    temp.y=graph[position].y;
    path.push_front(temp);
    while(1)
    {
    	temp.x=graph[previous[position]].x;
   		temp.y=graph[previous[position]].y;	
    	path.push_front(temp);
    	if(previous[position]==s) break;
    	position=previous[position];
    }
    fstream file;
    file.open("path.txt",fstream::out);
    for (int i = 0; i < path.size(); ++i)
    {
    	if(i==path.size()-1) file<<"("<<path[i].x<<","<<path[i].y<<")";
    	else
    	//file<<"("<<path[i].x<<","<<path[i].y<<")"<<"---->";
    }
}


int check_obstacle(botcoordinate i, botcoordinate j);
int check_obstacle_spl(botcoordinate i, botcoordinate j);
int build_graph();
void show_graph();
void build_path(int);
int dist(botcoordinate a1, botcoordinate a2);

int main()
{
	for(int i=0; i<nbot; i++)
	{
		//cout<<"Enter coordinates and id of bot "<<i+1<<endl;
		botcoordinate data;
		cin>>data.x>>data.y>>data.botid;
		list.push_back(data);		
	}


	botcoordinate origin,end;
	//cout<<"Enter coordinates and id of the origin bot"<<endl;
	cin>>origin.x>>origin.y>>origin.botid;
	//cout<<"Enter coordinates and id of the endpoint"<<endl;
	cin>>end.x>>end.y>>end.botid;
	
	for(int i=0; i<nbot; i++)
	{
		botcoordinate data[4];
		int x = list[i].x;
	    int y = list[i].y;
	    int id = list[i].botid;

	    data[0].x = x + R;
	    data[1].x = x + R;
	    data[2].x = x - R;
	    data[3].x = x - R;

	    data[0].y = y + R;
	    data[1].y = y - R;
	    data[2].y = y + R;
	    data[3].y = y - R;

	    data[0].botid = list[i].botid;
	    data[1].botid = list[i].botid;
	    data[2].botid = list[i].botid;
	    data[3].botid = list[i].botid;

	    for (int j = 0; j < 4;j++)
	    {
	        data[j].botid = id;
	        vertex.push_back(data[j]);
	    }
	}
	vertex.push_back(origin);
	vertex.push_back(end);

	int check =build_graph();
	show_graph();
	build_path(check);

	return 0;
}

int dist(botcoordinate a1, botcoordinate a2)
{
	return abs(a1.x-a2.x)+abs(a1.y-a2.y);
}

int build_graph()
{
	file.open ("graph.txt");
	for(int i=0; i<vertex.size(); i++)
	{
		botcoordinate *b;
		b = new botcoordinate;
		b->x = vertex[i].x;
		b->y = vertex[i].y;
		graph.push_back(*b);		
	}
	
	//checking direct link between origin and end
	if(check_obstacle(vertex[vertex.size()-2],vertex[vertex.size()-1])==0)
	{
		
		vertex[vertex.size()-1].edgewt=dist(vertex[vertex.size()-1],vertex[vertex.size()-2]);
		graph[vertex.size()-2].edgev.push_back(vertex[vertex.size()-1]);
		vertex[vertex.size()-2].edgewt=dist(vertex[vertex.size()-1],vertex[vertex.size()-2]);
		graph[vertex.size()-1].edgev.push_back(vertex[vertex.size()-2]);
		return 0;
	}

	//
	for(int i=0; i<(vertex.size()-2); i++)
	{
		for(int j=0; j<(vertex.size()-2); j++)
		{
			if(check_obstacle(vertex[i],vertex[j])==0 && j!=i)
			{
				vertex[j].edgewt=dist(vertex[i],vertex[j]);
				graph[i].edgev.push_back(vertex[j]);
			}
		}
	}

	//Add program to accomodate origin and end
	for(int i=(vertex.size()-2); i<vertex.size(); i++)
	{
		for(int j=0; j<vertex.size()-2; j++)
		{
			if(check_obstacle(vertex[i],vertex[j])==0)
			{
				vertex[j].edgewt=dist(vertex[i],vertex[j]);
				vertex[i].edgewt=dist(vertex[i],vertex[j]);
				graph[i].edgev.push_back(vertex[j]);
				graph[j].edgev.push_back(vertex[i]);
			}
		}
	}
	return 1;
}

void build_path(int check)
{
	//cout<<"Dijkstra's"<<endl;
	dijkstra(vertex.size()-2,vertex.size()-1);
}

void show_graph()
{
	
	for(int i=0; i<graph.size(); ++i)
	{
		file<<graph[i].x<<","<<graph[i].y<<"---"<<endl;
		for(int j=0; j<graph[i].edgev.size(); j++)
		{
			file<<"("<<graph[i].edgev[j].x<<","<<graph[i].edgev[j].y<<")"<<"-"<<graph[i].edgev[j].edgewt<<"  ";
		}
		file<<endl;		
	}
	file.close();
}

int check_obstacle(botcoordinate a1, botcoordinate a2)
{
	int flag=0,flag1=0;
	botcoordinate temp;

	if(a1.botid==a2.botid)
	{
		if((abs(a1.x-a2.x)+abs(a1.y-a2.y))==2*R)
			return 0;
		else return -1;	
	}

	if(a1.x==a2.x)
	{
		for(int i=0; i<nbot; i++)
		{
			if(a1.y>a2.y)
			{
				temp=a1;
				a1=a2;
				a2=temp;
			}

			if(((i+1)==a1.botid)||((i+1)==a2.botid))
			{
				if((i+1)==a1.botid)
				{
					if(a1.y==vertex[4*i+3].y)
						return -1;
					continue;
				}

				else
				{
					if(a2.y==vertex[4*i].y)
						return -1;
					continue;					
				}
			}

			if((vertex[4*i].x<a1.x)&&(vertex[4*i+3].x>a1.x))
				return 0;

			if((vertex[4*i].y>a1.y)&&(vertex[4*i+1].y<a2.y))
			{
				if(abs((((vertex[4*i+1].x+vertex[4*i+2].x)/2)+((vertex[4*i+1].y+vertex[4*i+2].y)/2))-((a1.x+a2.x)/2+(a1.y+a2.y)/2))>=15)
					continue;
				return -1;
			}
		}
		return 0;
	}

	if(a1.x>a2.x){
		temp=a1;
		a1=a2;
		a2=temp;
	}

	for(int i=0; i<nbot; ++i)
	{		
		if(((i+1)!=a1.botid)&&((i+1)!=a2.botid))
		{
			if(((a2.y-a1.y)*1.0/(a2.x-a1.x))>=0)   
			{
				flag = (float)(a2.y-a1.y)/(float)(a2.x-a1.x)*(a2.x-vertex[4*i+1].x)-(a2.y-vertex[4*i+1].y);
				flag1 = (float)(a2.y-a1.y)/(float)(a2.x-a1.x)*(a2.x-vertex[4*i+2].x)-(a2.y-vertex[4*i+2].y);
				if((flag1==0)||(flag==0))
				{
					flag1=0;
					flag=-1;
				}
				else
				{
					flag1 = flag1/abs(flag1); 
					flag = flag/abs(flag);
				}				
				if(flag!=flag1)
				{
					if(abs((((vertex[4*i+1].x+vertex[4*i+2].x)/2)+((vertex[4*i+1].y+vertex[4*i+2].y)/2))-((a1.x+a2.x)/2+(a1.y+a2.y)/2))>=15)
						continue;
					return -1;
				}
			}

			else
			{
				flag = (float)(a2.y-a1.y)/(float)(a2.x-a1.x)*(a2.x-vertex[4*i].x)-(a2.y-vertex[4*i].y);
				flag = flag/abs(flag);
				flag1 = (float)(a2.y-a1.y)/(float)(a2.x-a1.x)*(a2.x-vertex[4*i+3].x)-(a2.y-vertex[4*i+3].y);
				flag1 = flag1/abs(flag1);
				if(flag!=flag1)
					return -1;				
			}
		}

		if((i+1==a1.botid)||(i+1==a2.botid))
		{			
			if(i+1==a1.botid)
			{
				if(((a2.y-a1.y)*1.0/(a2.x-a1.x))>0)   
				{
					if((a1.y==vertex[4*i+3].y)&&(a1.x==vertex[4*i+3].x))    
						return -1;
					continue;
				}
				else if(((a2.y-a1.y)*1.0/(a2.x-a1.x))<0)
				{
					if((a1.y==vertex[4*i+2].y)&&(a1.x==vertex[4*i+2].x))     
						return -1;
					continue;
				}
				else
				{
					if((a1.y==vertex[4*i+3].y)&&(a1.x==vertex[4*i+3].x))     
						return -1;
					if((a1.y==vertex[4*i+2].y)&&(a1.x==vertex[4*i+2].x))     
						return -1;
				}
			}

			if(i+1==a2.botid)
			{
				if(((a2.y-a1.y)*1.0/(a2.x-a1.x))>0)   
				{
					if((a2.y==vertex[4*i].y)&&(a2.x==vertex[4*i].x))    
						return -1;
					continue;

				}
				else if(((a2.y-a1.y)*1.0/(a2.x-a1.x))<0)
				{
					if((a2.y==vertex[4*i+1].y)&&(a2.x==vertex[4*i+1].x))    
						return -1;
					continue;
				}
				else
				{
					if((a2.y==vertex[4*i].y)&&(a2.x==vertex[4*i].x))     
						return -1;
					if((a2.y==vertex[4*i+1].y)&&(a2.x==vertex[4*i+1].x))     
						return -1;
				}				
			}
		}	
	}
	return 0;	
}