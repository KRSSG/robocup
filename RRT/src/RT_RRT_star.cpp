#include "RT_RRT_star.hpp"

namespace rrt {

  template <class T>
  RT_RRT<T>::RT_RRT(Utils::Point<T> Xa, Utils::Point<T> Xg) {
    // TODO : set these parameters
    // TODO : Set all parameters in one scale
    goal_radius = 20.0;
    obstacle_radius = 20.0;
    search_radius = 20.0;
    node_thresh = 160.0;
    epsilon_radius = 20.0;
    k_max = 20.0;

    width = 6000;
    length = 4000;

    current_parent_idx = 0;
    this->Xa = Xa;
    this->Xg = Xg;
    tree.push_back(std::make_pair(Xa, 0));
    ++current_parent_idx;
  }

  template <class T>
  RT_RRT<T>::RT_RRT(T goal_radius, T obstacle_radius, T search_radius,
    T node_thresh, T epsilon_radius, long int k_max) {

    this->goal_radius = goal_radius;
    this->obstacle_radius = obstacle_radius;
    this->search_radius = search_radius;
    this->node_thresh = node_thresh;
    this->epsilon_radius = epsilon_radius;
    this->k_max = k_max;
    width = 6;
    length = 9;
    current_parent_idx = 0;
  }

  template <class T>
  void RT_RRT<T>::update_epsilon_radius() {
    size_t total_nodes = this->RT_RRT<T>::tree.size();

  }

  template <class T>
  std::vector<int > RT_RRT<T>::find_near_nodes(Utils::Point<T> query) {

    std::pair<unsigned int, unsigned int> grid_idx = this->Grid_Id(query);
    std::vector<int > nearest_points;
    for (int i = 0; i < grid_nodes[grid_idx].size(); ++i) {
        this->update_epsilon_radius();
        if (this->dist(query, tree[grid_nodes[grid_idx][i]].first) < this->epsilon_radius) {
          nearest_points.push_back(i);
        }
      }
    return nearest_points;
  }

  template <class T>
  std::pair<Utils::Point<T>, Utils::Point<T> > RT_RRT<T>::add_node_to_tree(Utils::Point<T> rand) {
    // std::cout<<"Tree size is: "<<tree.size()<<std::endl;
    // Utils::Point<T> closest = closest_node(rand);
    std::vector<int > x_near = find_near_nodes(rand);
    Utils::Point<T> new_point;
    int parent_idx = -1;
    if (x_near.size())
    {
      // std::cout<<"Here if\n";
      Utils::Point<T> x_min = tree[x_near[0]].first;
      parent_idx = x_near[0];

      double theta = atan2(rand.y-tree[x_near[0]].first.y,rand.x-tree[x_near[0]].first.x);
      new_point.x=tree[x_near[0]].first.x+node_thresh*cos(theta);
      new_point.y=tree[x_near[0]].first.y+node_thresh*sin(theta);

      double c_min = cost(x_min, 0, 0).first + dist(x_min, rand);
      for (int i = 1; i < x_near.size(); ++i) {
        double c_new = cost(tree[x_near[i]].first,1,i).first + dist(tree[x_near[i]].first, rand);


        if (c_new < c_min) {
          // std::cout<<"Here 2\n";
          double theta = atan2(rand.y-tree[x_near[i]].first.y,rand.x-tree[x_near[i]].first.x);
          new_point.x=tree[x_near[i]].first.x+node_thresh*cos(theta);
          new_point.y=tree[x_near[i]].first.y+node_thresh*sin(theta);

          if (line_path_obs(tree[x_near[i]].first, new_point)){
            c_min = c_new;
            x_min = tree[x_near[i]].first;
            parent_idx = i;
          }
        }
      }
    }
    else
    {
      // std::cout<<"Here else with tree size: "<<tree.size()<<std::endl;
      Utils::Point<T> x_min = tree[0].first;
      parent_idx = 0;

      double theta = atan2(rand.y-tree[0].first.y,rand.x-tree[0].first.x);
      new_point.x=tree[0].first.x+node_thresh*cos(theta);
      new_point.y=tree[0].first.y+node_thresh*sin(theta);

      double c_min = dist(tree[0].first, rand);
      // std::cout<<"Cost0: "<<c_min<<" from the point "<<tree[0].first.x<<","<<tree[0].first.y<<"\n";
      for (int i = 1; i < tree.size(); ++i) {
        double c_new = dist(tree[i].first, rand);
        // std::cout<<"Cost"<<i<<": "<<c_new<<" from the point "<<tree[i].first.x<<","<<tree[i].first.y<<"\n";
        if (c_new < c_min) {

          double theta = atan2(rand.y-tree[i].first.y,rand.x-tree[i].first.x);
          new_point.x=tree[i].first.x+node_thresh*cos(theta);
          new_point.y=tree[i].first.y+node_thresh*sin(theta);

          if (line_path_obs(tree[i].first, new_point)){
            c_min = c_new;
            x_min = tree[i].first;
            parent_idx = i;
          }
        }
      }
    }
    
    
    // assert(parent_idx != -1);
    tree.push_back(std::make_pair(new_point, parent_idx));
    add_node_to_grid(new_point, tree.size());
    // std::cout<<"Index: "<<parent_idx<<std::endl;
    return std::make_pair(new_point, tree[parent_idx].first);
  }

  template <class T>
  double RT_RRT<T>::dist(Utils::Point<T> first, Utils::Point<T> second)
  {
    return sqrt(pow(first.x-second.x,2)+pow(first.y-second.y,2));
  }

  template <class T>
  bool RT_RRT<T>::add_node_to_grid(Utils::Point<T> point, int new_idx) {
    std::pair<int, int> point_grid_idx = Grid_Id(point);

    if (grid_nodes[point_grid_idx].size() < k_max) {
      grid_nodes[point_grid_idx].push_back(new_idx);
      return true;
    }
    return false;
   }

  template <class T>
  void RT_RRT<T>::cube_round(float xyz[], float rxyz[])
  {
    float rx = nearbyint(xyz[0]);
    float ry = nearbyint(xyz[1]);
    float rz = nearbyint(xyz[2]);

    float x_diff = fabs(rx - xyz[0]);
    float y_diff = fabs(ry - xyz[1]);
    float z_diff = fabs(rz - xyz[2]);

    if ((x_diff > y_diff) && (x_diff > z_diff))
        rx = -ry-rz;
    else if (y_diff > z_diff)
        ry = -rx-rz;
    else
        rz = -rx-ry;
    rxyz[0] = rx; rxyz[1] = ry; rxyz[2] = rz;
  }

  template <class T>
  std::pair<int, int > RT_RRT<T>::cube_to_axial(float rxyz[])
  {
      int q = rxyz[0];
      int r = rxyz[2];
      return std::pair<int, int >(q, r);
  }

  template <class T>
  void RT_RRT<T>::axial_to_cube(float q, float r, float xyz[])
  {
    xyz[0] = q;
    xyz[2] = r;
    xyz[1] = -xyz[0]-xyz[2];
  }

  template <class T>
  std::pair<int, int > RT_RRT<T>::hex_round(float q,float r)
  {
    float xyz[3];
    RT_RRT<T>::axial_to_cube(q,r,xyz);
    float rxyz[3];
    RT_RRT<T>::cube_round(xyz, rxyz);
    return RT_RRT<T>::cube_to_axial(rxyz);
  }

  template <class T>
  std::pair<int, int> RT_RRT<T>::Grid_Id(Utils::Point<T> gride_idx, int size)
  {
    //size is edge length of hexagon
    float x = gride_idx.x, y = gride_idx.y;
    float q = (x * sqrt(3)/3 - y / 3) / size;
    float r = (y * 2/3) / size;
    std::pair<int, int> here = RT_RRT<T>::hex_round(q,r);
    return here;
  }

  template <class T>
  std::pair<double,Utils::Point<T> > RT_RRT<T>::cost(Utils::Point<T> child, int count, int k)
  {
            // k is location in the tree
            if (child==RT_RRT<T>::Xa)
              return std::pair<double,Utils::Point<T> > (0,RT_RRT<T>::Xa);
            std::pair<double,Utils::Point<T> > here;
            int j=0;
            if (count==0)
            {
              for(;j<RT_RRT<T>::tree.size();j++)
              {
                if(RT_RRT<T>::tree[j].first==child)
                {
                  // std::cout<<"broken\n";
                  break;
                }
              }
              // std::cout<<"j is: "<<j<<" and tree size is: "<<tree.size()<<std::endl;
              if (RT_RRT<T>::line_path_obs(child,(RT_RRT<T>::tree[RT_RRT<T>::tree[j].second].first)))
                here = cost(tree[tree[j].second].first,1,tree[j].second);
              else
                return std::pair<double,Utils::Point<T> > (std::numeric_limits<double>::infinity(),RT_RRT<T>::tree[RT_RRT<T>::tree[j].second].first);
            }
            else
            {
              // std::cout<<"No chance of breaking\n";
              if (RT_RRT<T>::line_path_obs(child,(RT_RRT<T>::tree[RT_RRT<T>::tree[k].second].first)))
                here = cost(tree[tree[k].second].first,1,tree[k].second);
              else
                return std::pair<double,Utils::Point<T> > (std::numeric_limits<double>::infinity(),RT_RRT<T>::tree[RT_RRT<T>::tree[j].second].first);
            }

            if (count)
              return std::pair<double,Utils::Point<T> >  (dist(child,RT_RRT<T>::tree[j].second)+here.first,here.second);
            else
              return std::pair<double,Utils::Point<T> >  (dist(child,RT_RRT<T>::tree[k].second)+here.first,RT_RRT<T>::tree[RT_RRT<T>::tree[j].second].first);
  }

  template <class T>
  void RT_RRT<T>::rewire_node(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > &Q)
  {
    Utils::Point<T> me = Q.pop();
    std::pair<double,Utils::Point<T> > now = cost(me);
    double cost = now.first;
    Utils::Point<T> parent = now.second;
    std::vector<Utils::Point<T> > neighbours= find_near_nodes(Q);
    for(size_t i = 0; i < neighbours.size(); i++)
        Q.push(neighbours[i]);
    for (int i=0;i<neighbours.size();i++)
    {
      Utils::Point<T> neighbour = neighbours[i];
      if (cost > cost - dist(me,parent) + dist(parent,neighbour) + dist(neighbour,me))
      {
            for(int j=0;j<RT_RRT<T>::tree.size();j++)
            {
              if(RT_RRT<T>::tree[j].first==me)
                RT_RRT<T>::tree[j].second = neighbour[i];
            }
      }
    }
  }

  template <class T>
  void RT_RRT<T>::rewire_root(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs)
  {
    int j;
    std::vector<Utils::Point<T> > neighbours= find_near_nodes(RT_RRT<T>::Xa);
    for(size_t i = 0; i < neighbours.size(); i++)
        Qs.push(neighbours[i]);
    rewire_node(Qs);
  }

  template <class T>
  bool RT_RRT<T>::line_path_obs(Utils::Point<T> p1, Utils::Point<T> p2)
  {
        // RT_RRT<T>::bool obstacle_here(int, int);
        // std::cout<<"Here!\n";
        int x1=p1.x, x2=p2.x, y1=p1.y,y2=p2.y;
        float x=x1, count;
        try
        {

                int m=float(y2-y1)/(x2-x1);
                if (m==0) throw 20;
                int c=y2-m*x2;
                count = fabs(1.0/m);
                if (count>1) count=1;
                if (count<-1) count=-1;
                if (x2<x1) count*=-1;
                // std::cout<<"\nm is "<<m<<" and count is: "<<count<<"\n";
                while (1)
                {
                    x+=count;
                    int y=m*x+c;
                    if ((count>0 and x>=x2) || (count<0 and x<=x2))
                    {
                        // std::cout<<"Return true from try\n";
                        return true;
                    }
                    else
                    {
                // std::cout<<"\nm is "<<m<<" and count is: "<<count<<"\n";
                //       std::cout<<std::endl<<"x: "<<x<<" x2: "<<x2<<std::endl;
                //       std::cout<<"count: "<<count<<std::endl;
                    }
                    if (obstacle_here(x,y))
                    {
                        // std::cout<<"Return false from try\n";
                        return false;
                    }
                }
        }
        catch(int e)
        {
                count=1;
                int y=y1;
                if (y2<y1) count*=-1;
                while (1)
                {
                    y+=count;
                    if ((count>0 and y>=y2) || (count<0 and y<=y2))
                    {
                      // std::cout<<"Return true from catch\n";
                        return true;
                    }
                    if (RT_RRT<T>::obstacle_here(x,y))
                    {
                      // std::cout<<"Return false from catch\n";
                        return false;
                    }
                }
        }

  }

  template <class T>
  bool RT_RRT<T>::obstacle_here(int x, int y)
  {
    for (int i=0;i<RT_RRT<T>::Xobs.size();i++)
    {
      Utils::Point<T> pratham;
      pratham.x = x; pratham.y = y;
      Utils::Point<T> dwitiya;
      dwitiya.x = RT_RRT<T>::Xobs[i].x; dwitiya.y = RT_RRT<T>::Xobs[i].y;
      if (RT_RRT<T>::dist(pratham,dwitiya)<RT_RRT<T>::obstacle_radius)
        return true;
    }
    return false;
  }

  template <class T>
  bool RT_RRT<T>::found_in_grid(std::pair<int, int> grid_id, Utils::Point<T> P)
  {
    try
    {
      std::vector<Utils::Point<T> > list_of_nodes = RT_RRT<T>::grid_nodes[grid_id];
      for (int i=0; i<list_of_nodes.size(); i++)
      {
        int now = list_of_nodes[i];
        if ((RT_RRT<T>::tree[now].first).first == P.first && (RT_RRT<T>::tree[now].first).second == P.second)
          return true;
      }
    }
    catch (int e)
    {}
    return false;
  }

  template <class T>
  Utils::Point<T> RT_RRT<T>::closest_node(Utils::Point<T> rand)
  {
    // std::cout<<"In close node\n";
    return  RT_RRT<T>::cost(rand).second;
  }

}
