/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_INCLUDE_RAPIDRANDOMTREE_H
#define RRT_INCLUDE_RAPIDRANDOMTREE_H

#include <vector>
#include "Eigen/Dense"

#define EPSILON       0.2f
#define BOUNDARYWIDTH 5.0f
#define BOUNDAYHEIGHT 5.0f

namespace rrt {

typedef Eigen::Matrix<float, 2, 1> vector2f1;

struct node {
  bool initialized;
  vector2f1 location_m;
  uint64_t id;
  std::vector<uint64_t> neighbors;
  node();
};

class rapidRandomTree {
public:
  // Constructor
  rapidRandomTree(const std::string& treeName_, float robotRadius_);
  // Deconstructor
  ~rapidRandomTree();

  /**
   * @brief     grows tree towards a random point
   * @return    returns new point
   */
  vector2f1 growTreeTowardsRandom();

  /**
   * @brief     grows tree towards specified point
   * @param     point to grow tree towards
   * @return    none
   */
  void growTreeTowardsPoint(vector2f1& setPt);

  bool goalReached() const {return reachedGoalPoint;}
  vector2f1 findPointOnLine(vector2f1& startPt, vector2f1& endPt, bool toRandom);
  static float distance(vector2f1 p1, vector2f1 p2);
  static float get_random(float lowerBound, float upperBound);
  uint64_t getIdOfLastPoint();
  vector2f1 getCoordinateOfLastNode() const {return lastNodeCoordinate;}
  vector2f1 getTreeStart();
  std::vector<node> getTree() const {return tree;}
  node getConnectingNeighbor() const {return connectingNeighbor;}

private:
  node createNode(vector2f1 newPt);
  uint64_t findClosestNeighbor(const vector2f1& randomPt);
  void setUpRobotModel();
  static void setUpObjects();
  bool collisionDetection(const vector2f1& point);
  void setConnectingNeighbor(node& leaf);

  node connectingNeighbor;
  vector2f1 lastNodeCoordinate;

  std::string treeName;
  std::vector<node> tree;
  float robotRadius;
  bool treeIsValid;
  bool reachedGoalPoint;
};
} //  end namespace rrt

#endif //RRT_INCLUDE_RAPIDRANDOMTREE_H
