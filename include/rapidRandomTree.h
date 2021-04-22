/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_INCLUDE_RAPIDRANDOMTREE_H
#define RRT_INCLUDE_RAPIDRANDOMTREE_H

#include <vector>

#define EPSIOLON      0.2
#define BOUNDARYWIDTH 5.0
#define BOUNDAYHEIGHT  5.0

namespace rrt {

struct node {
  struct location {
    float x_m;
    float y_m;
  } coordinate{};
  uint64_t id;
  std::vector<uint64_t> neighbors;
  node();
};

class rapidRandomTree {
public:
  // Constructor
  rapidRandomTree(const std::string& treeName_);
  // Deconstructor
  ~rapidRandomTree();

  /// Create a random point
  /// Find closest neighbor
  /// Grow tree from closest neighbor towards random point by distance epsiolon
  /// Add new point to tree
  /// returns xy-coordinates of new point
  std::pair<float, float> growTreeTowardsRandom();

  /// Grows tree towards specified point
  void growTreeTowardsPoint(float x_m, float y_m);

  static float get_random(float lowerBound, float upperBound);
  static float distance(float x1_m, float y1_m, float x2_m, float y2_m);

private:
  node createNode(float x_m, float y_m);
  uint64_t findClosestNeighbor(float x_m, float y_m);
  std::string treeName;
  std::vector<node> tree;
  bool treeIsValid;
};
} //  end namespace rrt

#endif //RRT_INCLUDE_RAPIDRANDOMTREE_H
