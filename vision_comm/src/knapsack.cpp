#include<iostream>

using namespace std;

struct A{
	int w,c, count, index;
};
int maxm_cost(vector<A> &objects_sorted, vector<A> &objects,int n, int B)
{
	int cost[B+1]={0}, index_c[B+1]={0};
	int i=objects_sorted[0].w, total_weight=0;

	while(i<=B){
		
			int max_cost=objects_sorted[0].c+cost[i- objects_sorted[0].w], index=0;
			for(int j=1;j<n;j++) {
			
				if(i- objects_sorted[j].w>=0) {
					if(objects_sorted[j].c+cost[i- objects_sorted[j].w]> max_cost) {
						max_cost=objects_sorted[j].c+cost[i- objects_sorted[j].w]> max_cost; index=j;
					}
				}
				else
					break;
			}
			index_c[i]=index;
			cost[i]=max_cost;	
			i++;
	}
	int j=B;
	while(j>=0)
	{
		objects_sorted[index_c[j]].count++;
		total_weight+=objects_sorted[index_c[j]].w;
		j-=objects_sorted[index_c[j]].w;
	}
	cout<<"\n\n+++ Maximum cost = "<<cost[B]<<endl;
	cout<<"+++ Weight = "<<total_weight<<endl;
	cout<<"+++ Counts\t: ";
	sort(objects_sorted.begin(), objects_sorted.end(), back_sort)
	for(j=0;j<n;j++)
		cout<<objects[j].count<<" ";
	return cost[B];
}

bool check(A l, A r)
{
	return l.w<r.w;
}

bool back_sort(A l, A r)
{
	return l.index<r.index;
}

int main()
{
	int n,B;
	vector<A> objects(n);
	vector<A> objects_sorted;

	cout<<"+++ n = "<<n<<endl;
	cout<<"+++ B = "<<B<<endl;
	cout<<"+++ Weights : ";
	for(int i=0;i<n;i++)
		{ cin>>objects[i].w; cout<<objects[i].w<<" "; objects[i].count=0;}

	cout<<endl<<"+++ Costs\t: ";

	for(int i=0;i<n;i++)
		{ cin>>objects[i].c; cout<<objects[i].c<<" "; objects[i].index=i;}

	objects_sorted=objects;
	sort(objects_sorted.begin(), objects_sorted.end(), check)
	int maxm_cost=calculate_cost(objects_sorted, objects, n, B);

	return 0;
}