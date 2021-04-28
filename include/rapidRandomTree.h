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

// fcl includes
#include "fcl/narrowphase/collision.h"
#include "fcl/math/geometry.h"

namespace rrt {

typedef Eigen::Matrix<float, 2, 1> vector2f1;

struct node {
  bool goalNode;
  vector2f1 location_m;
  uint64_t id;
  std::vector<uint64_t> neighbors;
  node();
};

struct objectNode {
  uint64_t objectId;
  vector2f1 location_m;
  float width;
  float height;
  float orientation;
  objectNode();
};

class rapidRandomTree {
public:
  // Constructor
  rapidRandomTree(const std::string& treeName_, rapidRandomTree* otherTree, bool goalTree_);
  // Deconstructor
  ~rapidRandomTree();

  /**
   * @brief     grows tree towards a random point
   * @return    none
   */
  void growTreeTowardsRandom();

  /**
   * @brief     grows tree towards specified point
   * @param     point to grow tree towards
   * @return    none
   */
  void growTreeTowardsPoint(vector2f1& setPt);

  static float distance(vector2f1 p1, vector2f1 p2);
  static float get_random(float lowerBound, float upperBound);
  static uint64_t findClosestNeighbor(const vector2f1& randomPt, const std::vector<node>& searchTree);
  static vector2f1 projectToPointOnLine(vector2f1& startPt, vector2f1& endPt, float distanceToProject);
  static vector2f1 closestPointOnSegment(vector2f1& startPt, vector2f1& endPt, vector2f1& queryPt);
  static bool lineIntersection(vector2f1& intersectPt, rrt::vector2f1 A, rrt::vector2f1 B, rrt::vector2f1 C, rrt::vector2f1 D, float step_m);
  static bool collisionDetection(const vector2f1& point,
                                 std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>  robotModel,
                                 std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> objects,
                                 std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> walls);

  void setOtherTree(const std::vector<node>& otherTree_) {otherTree = otherTree_;}

  // Get functions of object class variables that are read only
  bool goalReached() const {return reachedGoalPoint;}
  vector2f1 getCoordinateOfLastNode() const {return lastNodeCoordinate;}
  vector2f1 getTreeStart();
  std::vector<node> getTree() const {return tree;}
  node getConnectingNeighbor() const {return connectingNeighbor;}
  objectNode getRobotModel() const {return robotInMap;}
  std::vector<objectNode> getObjects() const {return objectInMap;}
  std::vector<objectNode> getBorderWalls() const {return border;}
  std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> getObjectAndTransform() const {return objects;}
  std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> getWallsAndTransform() const {return walls;}
  std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f> getRobotAndTransform() const {return robotModel;}

private:
  node createNode(vector2f1 newPt);
  void setUpRobotModel();
  void setWalls();
  void placeRobotInMap();
  void setUpObjects();
  bool newObstacleCollisionDetection(objectNode& newObject);
  void setConnectingNeighbor(node& leaf);
  bool connectToNeighborSegment(vector2f1& queryPt, uint64_t neighbor);
  vector2f1 connectToOtherTree(vector2f1& point, uint64_t thisNeighbor);

  node connectingNeighbor;
  vector2f1 lastNodeCoordinate;
  vector2f1 initPoint;

  std::string treeName;
  std::vector<node> tree;       // This object's tree being expanded
  std::vector<node> otherTree;  // Dual tree trying to connect with

  objectNode robotInMap;
  std::vector<objectNode> border;
  std::vector<objectNode> objectInMap;

  float maxStepDistance_m;
  float boundaryWidth_m;
  float boundaryHeight_m;
  float maxObjectSize_m;
  float robotHeight_m;
  float robotWidth_m;

  bool reachedGoalPoint;

  uint64_t minNumberOfObjects;
  uint64_t numberOfObjects;

  std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>  robotModel;
  std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> objects;
  std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> walls;
};
} //  end namespace rrt

#endif //RRT_INCLUDE_RAPIDRANDOMTREE_H
