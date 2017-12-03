#ifndef KDTREE_H
#define KDTREE_H

#include <stdint.h>
#include <cstdlib>
#include "ssl_common/geometry.hpp"

//#define UP_DOWN_SEARCH

#define KD_MAX_POINTS (500)  // max KD tree points number  

#define KDTEMPLATE template <class T>
#define KDNODE     KDNode<T>
#define KDTREE     KDTree<T>

namespace Navigation
{
  KDTEMPLATE
  inline T distance2(const T *p1, const T *p2)
  {
    return ((p1[0] - p2[0]) * (p1[0] - p2[0])) + ((p1[1] - p2[1]) * (p1[1] - p2[1]));
  }

  KDTEMPLATE
  inline bool equal(const T *p1, const T *p2)
  {
    return (p1[0] == p2[0] && p1[1] == p2[1]);
  }

  // KDTreeNode template class implementation
  KDTEMPLATE
  class KDNode
  {
    // member functions
  public:
    uint8_t      axis;
    T            pt[2];
    KDNODE*      left;
    KDNODE*      right;
    KDNODE*      kdParent;
    KDNODE*      errtParent;
    bool         checked;
    unsigned int id;

    KDNode(const T *point, int axis);

    Vector2D<T> toVector2D() const;
    KDNODE*     Insert(const T *point);
    KDNODE*     FindParent(const T *point);
    KDNODE*     FindNode(const T *point);
  }; // KDNode

  KDTEMPLATE
  KDNODE::KDNode(const T *point, int axis) :
    axis(axis),
    left(NULL),
    right(NULL),
    kdParent(NULL),
    errtParent(NULL),
    checked(false),
    id(0)
  {
    pt[0] = point[0];
    pt[1] = point[1];
  }

  KDTEMPLATE
  Vector2D<T> KDNODE::toVector2D(void) const
  {
    return Vector2D<T>(pt[0], pt[1]);
  }

  // The FindParent() member function finds the parent of a target point
  KDTEMPLATE
  KDNODE* KDNODE::FindParent(const T *point)
  {
    KDNODE* parent;
    KDNODE* next = this;

    while (next)
    {
      int split = next->axis;
      parent = next;

      if (point[split] > next->pt[split])
        next = next->right;
      else
        next = next->left;
    }
    return parent;
  }

  // The FindMode() member function finds the node of a target point
  KDTEMPLATE
  KDNODE* KDNODE::FindNode(const T *point)
  {
    KDNODE* next = this;

    while (next)
    {
      if (equal(point, next->pt))
        return next;
      int split = next->axis;
    
      if (point[split] > next->pt[split])
        next = next->right;
      else
        next = next->left;
    }
    return NULL;  // Node corresponding to x0 not found in the tree
  }

  // Insert() function links the new node and rotates the split plane index
  KDTEMPLATE
  KDNODE* KDNODE::Insert(const T *point)
  {
    KDNODE* parent = FindParent(point);

    if (equal(point, parent->pt))
      return NULL;

    KDNODE* newNode = new KDNODE(point, parent->axis + 1 < 2 ? parent->axis + 1 : 0);
    newNode->kdParent = parent;

    if (point[parent->axis] > parent->pt[parent->axis])
      parent->right = newNode;
    else
      parent->left = newNode;
    return newNode;
  }

  KDTEMPLATE
  class KDTree
  {
  public:
    KDNODE* root;

    KDTree();

    ~KDTree();

    bool           add(const T *point);
    bool           add(const Vector2D<T>& point);
    bool           add(const T *point, const T *parent);
    bool           add(const Vector2D<T>& point, const Vector2D<T>& parent);

    KDNODE*        find_nearest(const T *point);

    inline  void   check_subtree(KDNODE* node, const T *point);
    inline  void   set_bounding_cube(KDNODE* node, const T *point);
#ifndef UP_DOWN_SEARCH
    inline KDNODE* search_parent(KDNODE* parent, const T *point);
    void           uncheck(void);
#endif // !UP_DOWN_SEARCH

    void           deleteTree(KDNODE* root);

  private:
    T        d_min;
    KDNODE*  nearest_neighbour;

    int      kd_id;

    KDNODE*  checkedNodes[KD_MAX_POINTS];
    int      checked_nodes;

    T        x_min[2], x_max[2];
    bool     max_boundary[2], min_boundary[2];
    int      n_boundary;
  };

  KDTEMPLATE
  KDTREE::KDTree() :
    root(NULL),
    kd_id(1)
  { }

  KDTEMPLATE
  KDTREE::~KDTree()
  {
    deleteTree(root);
  }

  KDTEMPLATE
  void KDTREE::deleteTree(KDNODE* root)
  {
    if (root != NULL)
    {
      deleteTree(root->left);
      deleteTree(root->right);
      delete root;
      root = NULL;
    }
  }

  KDTEMPLATE
  bool KDTREE::add(const T *point)
  {
    if (!root)
    {
      root = new KDNODE(point, 0);
      root->id = kd_id++;
    }
    else
    {
      KDNODE* pNode;
      if ((pNode = root->Insert(point)))
        pNode->id = kd_id++;
    }
    return (kd_id < KD_MAX_POINTS);
  }

  KDTEMPLATE
  bool KDTREE::add(const T point[2], const T parent[2])
  {
    if (!root)
    {
      root = new KDNODE(point, 0);
      root->id = kd_id++;
    }
    else
    {
      KDNODE* pNode;
      if ((pNode = root->Insert(point)))
      {
        pNode->id = kd_id++;
        KDNODE* par = root->FindNode(parent);
        pNode->errtParent = par;
      }
    }
    return (kd_id < KD_MAX_POINTS);
  }

  KDTEMPLATE
  bool KDTree<T>::add(const Vector2D<T>& point)
  {
    int pt[2] = {point.x, point.y};
    return add(pt);
  }

  KDTEMPLATE
  bool KDTree<T>::add(const Vector2D<T>& point, const Vector2D<T>& parent)
  {
    int pt[2] = {point.x, point.y};
    int pp[2] = {parent.x, parent.y};
    return add(pt, pp);
  }

  /* The up-down function will chech too much halfspaces when the tree root and the target point
   * are too far away. Thus the search function below which start from the target parent and
   * move up in the tree - down-up search.
   * It uses the Parent link and goes up until the target is closed from all sides with large enough
   * hyperrectangle, or if the tree root is reached
   */
  KDTEMPLATE
  KDNODE* KDTREE::find_nearest(const T *point)
  {
    if (!root)
      return NULL;

    checked_nodes = 0;

    KDNODE* parent = root->FindParent(point);
    nearest_neighbour = parent;
    d_min = distance2(point, parent->pt);

    if (equal(point, parent->pt))
      return nearest_neighbour;

#ifdef UP_DOWN_SEARCH
    check_subtree(root, point);
#else
    search_parent(parent, point);
    uncheck();
#endif // !UP_DOWN_SEARCH

    return nearest_neighbour;
  }

  KDTEMPLATE
  void KDTREE::check_subtree(KDNODE* node, const T *point)
  {
    if (node == NULL || node->checked)
      return;

    checkedNodes[checked_nodes++] = node;
    node->checked = true;
    set_bounding_cube(node, point);

    int dim = node->axis;
    T d = node->pt[dim] - point[dim];

    if (d * d > d_min)
    {
      if (node->pt[dim] > point[dim])
        check_subtree(node->left, point);
      else
        check_subtree(node->right, point);
    }
    /* If the distance from the key to the current value is
     * less than the nearest distance, we still need to look
     * in both directions.
     */
    else
    {
      check_subtree(node->left, point);
      check_subtree(node->right, point);
    }
  }

  KDTEMPLATE
  void KDTREE::set_bounding_cube(KDNODE* node, const T *point)
  {
    if (!node)
      return;

    int d = 0;
    T dp;

    for (int k = 0; k < 2; ++k)
    {
      dp = node->pt[k] - point[k];
      if (dp > 0)
      {
        dp *= dp;
        if (!max_boundary[k])
        {
          if (dp > x_max[k])
            x_max[k] = dp;
          if (x_max[k] > d_min)
          {
            max_boundary[k] = true;
            ++n_boundary;
          }
        }
      }
      else
      {
        dp *= dp;
        if (!min_boundary[k])
        {
          if (dp > x_min[k])
            x_min[k] = dp;
          if (x_min[k] > d_min)
          {
            min_boundary[k] = true;
            ++n_boundary;
          }
        }
      }
      d += dp;
      if (d > d_min)
        return;
    }

    if (d < d_min)
    {
      d_min = d;
      nearest_neighbour = node;
    }
  }

#ifndef UP_DOWN_SEARCH
  KDTEMPLATE
  KDNODE* KDTREE::search_parent(KDNODE* parent, const T *point)
  {
    for (int k = 0; k < 2; ++k)
    {
      x_min[k] = x_max[k] = 0;
      max_boundary[k] = min_boundary[k] = 0;
    }

    n_boundary = 0;

    KDNODE* search_root = parent;

    // BUG Sometimes the loop below runs infinitely
    while (parent && n_boundary != 2 * 2)
    {
      check_subtree(parent, point);
      search_root = parent;
      parent = parent->kdParent;
    }

    return search_root;
  }

  KDTEMPLATE
  void KDTREE::uncheck(void)
  {
    for (int n = 0; n < checked_nodes; ++n)
      checkedNodes[n]->checked = false;
  }
#endif // !UP_DOWN_SEARCH
} // namespace Strategy

#endif // KDTREE_H
