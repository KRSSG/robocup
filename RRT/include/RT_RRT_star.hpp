#ifndef _RT_RRT_STAR_HPP_
#define _RT_RRT_STAR_HPP_

#include "utils.hpp"

namespace rrt {

  template <class T>
  class RT_RRT {
  private:

    // Main Tree
    std::vector<std::pair<Utils::Point<T>, int > > tree;

    // Queue at the Point of addition of the Point to the tree
    std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qr;
    // Queue at the root of the tree
    std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs;

    // Grid based subset of nodes for grid based search
    std::map<std::pair<int, int> , std::vector<int > > grid_nodes;

    // Position of the agent
    Utils::Point<T> Xa;

    // Position of the goal
    Utils::Point<T> Xg;

    // Position of all the obtacles
    std::vector<Utils::Point<T> > Xobs;

    // Path is planned if a Point is added to tree within this radius of goal Point
    T goal_radius;

    // All the nodes within this radius of an obstacle are given infinite cost
    T obstacle_radius;

    // We care about obstacles within this radius from the agent
    T search_radius;

    // Minimum distance between any two nodes in the tree
    T node_thresh;

    // Used for finding the subset of nodes near to a given node
    T epsilon_radius;

    // Grid density
    long int k_max;

    // Useful parameters for setting some search parameters
    unsigned int width, length;

    // Current parent id in the main tree
    int current_parent_idx;

  public:
    RT_RRT(Utils::Point<T> Xa, Utils::Point<T> Xg);

    RT_RRT(T goal_radius, T obstacle_radius, T search_radius, T node_thresh,
      T epsilon_radius, long int k_max);

    /** @brief Compute the distance between two nodes
    */

    double dist(Utils::Point<T>, Utils::Point<T>);


    /** @brief Return Grid Id
     */
    void cube_round(float*, float*);

    std::pair<int, int > cube_to_axial(float*);

    void axial_to_cube(float , float , float*);

    std::pair<int, int > hex_round(float,float);

    std::pair<int, int> Grid_Id(Utils::Point<T>, int size = 4);

    /** @brief Return cost
     */
    std::pair<double,Utils::Point<T> > cost(Utils::Point<T> child, int count=0, int k =-1);

    /** @brief Simultaneously expand and rewire the tree
     */
    void expand_rewire();  //left

    /** @brief Find all the nodes which are near to @p query
     */
    std::vector<int > find_near_nodes(Utils::Point<T> query);

    /** @brief Add a new Point to the tree
     */
    std::pair<Utils::Point<T>, Utils::Point<T> >  add_node_to_tree(Utils::Point<T> rand);

    /** @brief Rewire the tree from the latest node added to the tree
     */
    void rewire_node(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > &Q);

    /** @brief Rewire the tree from the agent's position
    */
    void rewire_root(std::queue<std::pair<Utils::Point<T>, Utils::Point<T> > > Qs);

    /** @brief Update the radius of neareset nodes prior to calling `find_near_nodes`
    */
    void update_epsilon_radius();

    /** @brief Find the closest node
    */
    Utils::Point<T> closest_node(Utils::Point<T> rand);  //left

    /** @brief Add a node to the grid
    */
    bool add_node_to_grid(Utils::Point<T> point, int new_idx);

    /** @brief find parent idx of the `son node`
    */
    int find_parent_idx(Utils::Point<T> son);

    /** @brief Verify if the line crosses any obstacle
    */
    bool line_path_obs(Utils::Point<T>, Utils::Point<T>);
    bool obstacle_here(int, int);
    /** @brief Verify if the point is present in the grid
    */
    bool found_in_grid(std::pair< int, int>, Utils::Point<T>);
  };

}

#endif
