/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/
// /usr/include
#include <iostream>
#include <thread>
#include <cstdio>
#include <cmath>
#include <random>
#include <utility>

// fcl includes
#include "fcl/narrowphase/collision.h"
#include "fcl/math/geometry.h"

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"

namespace rrt {

using namespace fcl;

node::node() {
  initialized = false;
  location_m.setZero();
  id = 0;
  neighbors.clear();
}

objectNode::objectNode() {
  objectId = 0;
  location_m.setZero();
  width = 0;
  height = 0;
}

std::shared_ptr<Boxf> robotModel;
std::vector<std::pair<std::shared_ptr<Boxf>, Transform3f>> objects;

// Constructor
rapidRandomTree::rapidRandomTree(const std::string& treeName_, float robotRadius_) {
  treeName = treeName_;
  robotRadius = robotRadius_;
  setUpRobotModel();
  setUpObjects();
  vector2f1 startPt;
  bool collision = true;
  while(collision) {
    startPt.x() = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
    startPt.y() = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);
    collision = collisionDetection(startPt);
  }
  initPoint = startPt ;
  placeRobotInMap();
  std::cout << "Creating new tree " << treeName << " at point {x, y} = {" <<
    startPt.x() << ", " << startPt.y() << "}" << std::endl;

  tree.clear();
  tree.emplace_back(createNode(startPt));
  treeIsValid = true;
  reachedGoalPoint = false;
}

// De-Constructor
rapidRandomTree::~rapidRandomTree() {
  std::cout << "De-constructing tree " << treeName << std::endl;
}

vector2f1 rapidRandomTree::growTreeTowardsRandom() {
  vector2f1 randomPoint, newPoint;
  bool randomPointCollison;
  bool robotoCollision = true;
  uint64_t neighbor;

  while(robotoCollision) {
    // Find new point until random point does not collide
    randomPointCollison = true;
    while (randomPointCollison) {
      // Create a random point
      randomPoint.x() = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
      randomPoint.y() = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);

      // Check collision detector of random point
      randomPointCollison = collisionDetection(randomPoint);
    }

    // Find index of closest neighbor
    neighbor = findClosestNeighbor(randomPoint);

    // Grow tree from closest neighbor towards random point by distance epsilon
    newPoint = projectToPointOnLine(tree.at(neighbor).location_m, randomPoint, true);

    // Check collision detector of new point
    robotoCollision = collisionDetection(newPoint);
  }

  // Set node of neighbor to new point
  setConnectingNeighbor(tree.at(neighbor));
  // Make nearest neighbor point to new node.
  tree.at(neighbor).neighbors.emplace_back(tree.size());
  // Add new point to tree
  tree.emplace_back(createNode(newPoint));

  // returns xy-coordinates of new point
  return newPoint;
}

void rapidRandomTree::growTreeTowardsPoint(vector2f1& setPt) {
  vector2f1 newPoint;

  // Find closest neighbor
  auto neighbor = findClosestNeighbor(setPt);

  // Check if setPt can connect to nearest neighbor on a segment
  if(connectToNeighborSegment(setPt, neighbor)) {
    // Store node of connecting neighbor to new point
    setConnectingNeighbor(tree.at(neighbor));
    lastNodeCoordinate = newPoint;
    reachedGoalPoint = true;
    return;
  }

  // Grow tree from closest neighbor towards point by distance epsilon
  newPoint = projectToPointOnLine(tree.at(neighbor).location_m, setPt, false);

  // Check collision detector
  if(collisionDetection(newPoint)) {
    return;                               // return without adding new point to tree
  }

  // Store node of connecting neighbor for get function
  setConnectingNeighbor(tree.at(neighbor));
  lastNodeCoordinate = newPoint;

  // Add new point to tree
  if(reachedGoalPoint) {
    return;
  }

  // Add new point to tree
  tree.emplace_back(createNode(newPoint));
  // Make new node point to the nearest neighbor.
  // This will allow the start tree to point to the connecting node
  //  in the goal tree and then find the goal point
  tree.back().neighbors.emplace_back(neighbor);
}

node rapidRandomTree::createNode(vector2f1 newPt) {
  node newNode;
  newNode.initialized = true;
  newNode.location_m.x() = newPt.x();
  newNode.location_m.y() = newPt.y();
  newNode.id = (uint64_t) tree.size();
  return newNode;
}

uint64_t rapidRandomTree::findClosestNeighbor(const vector2f1& randomPt) {
  if(tree.empty()) {
    std::cout << "Tree is empty. This value is invalid" << std::endl;
    treeIsValid = false;
    return 0;
  }

  if(tree.size() == 1) {
    return 0;                             // closest neighbor will be only node in tree
  }

  std::vector<float> distances;
  for(const auto& it : tree) {
    distances.emplace_back(distance(it.location_m, randomPt));
  }

  // pull the iterator with the smalles distance value
  auto it = std::min_element(distances.begin(), distances.end());
  auto index = std::distance(distances.begin(), it);  // get the index value of the vector at iterator
  return index;
}

vector2f1 rapidRandomTree::projectToPointOnLine(vector2f1& startPt, vector2f1& endPt, bool toRandom) {
  vector2f1 newPoint;
  float x;
  float y;

  // MAG of line segment from start to end
  float dy = endPt.y()-startPt.y();
  float dx = endPt.x()-startPt.x();
  if(dx == 0.f) {                         // Means we are on a vertical line
    x = endPt.x();
    if(dy > 0) {
      y = startPt.y() + EPSILON;
      if(y > endPt.y()) {                 // Do not want to set new point past endpoint
        y = endPt.y();
        reachedGoalPoint = !toRandom;
      }
    } else {
      y = startPt.y() - EPSILON;
      if(y < endPt.y()) {                 // Do not want to set new point past endpoint
        y = endPt.y();
        reachedGoalPoint = !toRandom;
      }
    }
    newPoint.x() = x;
    newPoint.y() = y;
    return newPoint;
  } else if(dy == 0.f) {                  // Means we are on a horizontal line
    y = endPt.y();
    if(dx > 0) {
      x = startPt[0] + EPSILON;
      if(x > endPt.x()) {                 // Do not want to set new point past endpoint
        x = endPt.x();
        reachedGoalPoint = !toRandom;
      }
    } else {
      x = startPt.x() - EPSILON;
      if(x < endPt.x()) {                 // Do not want to set new point past endpoint
        x = endPt.x();
        reachedGoalPoint = !toRandom;
      }
    }
    newPoint.x() = x;
    newPoint.y() = y;
    return newPoint;
  }

  float MAG = distance(startPt, endPt);
  float SLOPE = dy/dx;

  // determine if epsilon goes past Goal point
  double distanceFromEnd = MAG - EPSILON;
  if(distanceFromEnd > 0) {               // goal point not past end point
    double b = startPt[1] - SLOPE*startPt[0];
    // Use dist formula with goal.Y subbed in to get goal.X
    double c = b - startPt[1];
    double A = pow(SLOPE,2) + 1;
    double B = 2*c*SLOPE - 2*startPt[0];
    double C = pow(c,2) + pow(startPt[0],2) - pow(EPSILON,2);
    // determine points based on quadratic formula
    double Xg1 = (-B + sqrt(pow(B,2) - 4*A*C))/(2*A);
    double Xg2 = (-B - sqrt(pow(B,2) - 4*A*C))/(2*A);
    double Yg1 = SLOPE*Xg1 + b;
    double Yg2 = SLOPE*Xg2 + b;
    vector2f1 point1 = {Xg1, Yg1};
    vector2f1 point2 = {Xg2, Yg2};
    double goalToEnd1 = sqrt(pow((endPt[0]-point1[0]),2) +
        pow((endPt[1]-point1[1]),2));
    double goalToEnd2 = sqrt(pow((endPt[0]-point2[0]),2) +
        pow((endPt[1]-point2[1]),2));

    if(goalToEnd1 < goalToEnd2) {
      newPoint = point1;
    } else {
      newPoint = point2;
    }
  } else {                                // make end point the goal point
    newPoint = endPt;
    reachedGoalPoint = !toRandom;
  }

  return newPoint;
}

vector2f1 rapidRandomTree::closestPointOnSegment(vector2f1 &startPt, vector2f1 &endPt, vector2f1 &queryPt) {
  vector2f1 pathPt;
  vector2f1 edgeVec;
  edgeVec = endPt - startPt;

  //	1x2 	 * 	2x1 	= 1x1 constant
  // edgeVec' * edgeVec
  // In the case of arrays, a 1x2 keeps the same indexes as 2x1
  // Therefore transpose has the same output as input
  float edgeLengthSquared = edgeVec.x()*edgeVec.x() + edgeVec.y()*edgeVec.y();

  if(edgeLengthSquared == 0) {
    return startPt;	//return 2x1 closest point on a line segment
  }

  /*
    Consider the line extending the segment, parameterized as startPt + t (endPt - startPt).
    We find projection of point currentPos onto the line (dot product)
    It falls where t = [(currentPos-startPt) . (endPt-startPt)] / |endPt-startPt|^2
  */

  //  					1x2					 	*2x1		/C
  // t = (currentPos-startPt)'*edgeVec/edgeLengthSquared;
  vector2f1 temp = (queryPt - startPt);
  float temp2 = (temp.x()*edgeVec.x() + temp.y()*edgeVec.y())/edgeLengthSquared;

  // pathPt = startPt + t*edgeVec
  edgeVec.x() *= (float) temp2;
  edgeVec.y() *= (float) temp2;
  pathPt = startPt + edgeVec;

  if (temp2 < 0) {
    pathPt = startPt;
  } else if (temp2 > 1) {
    pathPt = endPt;
  }

  return pathPt;		// return 2x1
}

float rapidRandomTree::distance(vector2f1 p1, vector2f1 p2) {
  float dx = p1.x() - p2.x();
  float dy = p1.y() - p2.y();
  auto distance = (float) sqrt(pow(dx, 2) + pow(dy, 2));
  return distance;
}

float rapidRandomTree::get_random(float lowerBound, float upperBound) {
  // seed based on time in ms
  double currentTime = system::timerEvent::getRunTime_ms();   // returns epoch in ms
  static std::default_random_engine e((unsigned long) (currentTime));
  static std::uniform_real_distribution<> dis(lowerBound, upperBound);
  auto randomValue = (float) dis(e);
//  std::cout << "random value = " << randomValue << std::endl;
  return randomValue;
}

void rapidRandomTree::setUpRobotModel() {
  robotModel = std::make_unique<Boxf>(robotRadius, robotRadius, 0);
}

void rapidRandomTree::placeRobotInMap() {
  robotInMap.location_m = initPoint;
  robotInMap.width = robotRadius;
  robotInMap.height = robotRadius;
}

void rapidRandomTree::setUpObjects() {
  std::pair<std::shared_ptr<Boxf>, Transform3f> object;
  objectNode newObject;
  Transform3f tf;
  tf.setIdentity();
  tf.translation().z() = 0;
  tf.rotation().eulerAngles(0, 1, 2)[0] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[1] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[2] = 0;

  tf.translation().x() = 4.875;
  tf.translation().y() = 4.875;
  std::shared_ptr<Boxf> object1(new Boxf(0.25, 0.25, 0));
  object = {object1, tf};
  objects.emplace_back(object);
  newObject.location_m.x() = 4.875;
  newObject.location_m.y() = 4.875;
  newObject.height = 0.25;
  newObject.width = 0.25;
  newObject.objectId = 0;
  objectInMap.emplace_back(newObject);

  tf.translation().x() = 0.000;
  tf.translation().y() = 3.000;
  std::shared_ptr<Boxf> object2(new Boxf(0.50, 0.50, 0));
  object = {object2, tf};
  objects.emplace_back(object);
  newObject.location_m.x() = 0.000;
  newObject.location_m.y() = 3.000;
  newObject.height = 0.50;
  newObject.width = 0.50;
  newObject.objectId = 1;
  objectInMap.emplace_back(newObject);

  tf.translation().x() = -2.550;
  tf.translation().y() = -3.000;
  std::shared_ptr<Boxf> object3(new Boxf(0.50, 0.50, 0));
  object = {object3, tf};
  objects.emplace_back(object);
  newObject.location_m.x() = -2.550;
  newObject.location_m.y() = -3.000;
  newObject.height = 0.50;
  newObject.width = 0.50;
  newObject.objectId = 2;
  objectInMap.emplace_back(newObject);
}

bool rapidRandomTree::collisionDetection(const vector2f1& point) {
  if(objects.empty()) {
    return false;
  }

  Transform3f tfRobot;
  auto trans = tfRobot.translation();
  tfRobot.setIdentity();
  tfRobot.translation().x() = point.x();  // This places the robot at the point inserted
  tfRobot.translation().y() = point.y();  // This will tell if point is even viable for the robot to exist at
  tfRobot.translation().z() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).x() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).y() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).z() = 0;

  CollisionRequestf request;
  CollisionResultf result;
  CollisionObjectf object1(robotModel, tfRobot);
  for(const auto& it : objects) {
    CollisionObjectf object2(it.first, it.second);
    collide(&object1, &object2, request, result);
    if(result.isCollision()) {
//      std::cout << "Point = " << point.x() << ", " << point.y() << std::endl;
//      std::cout << "collided at " << it.second.translation().x() << ", " << it.second.translation().y() << std::endl;
      return true;                        // Point is not viable if collision is detected with any object
    }
  }

  return false;
}

vector2f1 rapidRandomTree::getTreeStart() {
  if(tree.empty()) {
    std::cout << "Tree for " << treeName << " is empty." << std::endl;
    vector2f1 emptyVector;
    emptyVector.setZero();
    return emptyVector;
  }

  return tree.front().location_m;
}

uint64_t rapidRandomTree::getIdOfLastPoint() {
  if(tree.empty()) {
    std::cout << "Returning invalid tree id" << std::endl;
    return UINTMAX_MAX;
  }
  return tree.back().id;
}

void rapidRandomTree::setConnectingNeighbor(node& leaf) {
  connectingNeighbor = leaf;
}

bool rapidRandomTree::connectToNeighborSegment(vector2f1 &queryPt, uint64_t neighbor) {
  node neighborNode = tree.at(neighbor);
  if(neighborNode.neighbors.empty()) {    // Neighbor node is a leaf node
    return false;                         // Will extend queryPt towards neighbor later
  }

  std::cout << std::endl << "Query Pt = " << queryPt << std::endl;
  for(auto it : neighborNode.neighbors) { // Check all segments of nodes connecting to neighbor
    auto nearestPointOnSegment = closestPointOnSegment(neighborNode.location_m, tree.at(it).location_m, queryPt);
    auto distanceToNearest = distance(nearestPointOnSegment, queryPt);
    std::cout << "Nearest Pt = " << nearestPointOnSegment << std::endl;
    std::cout << "Distance to Nearest Pt = " << distanceToNearest << std::endl << std::endl;
    if(distanceToNearest <= EPSILON) {
      std::cout << "Connecting to tree by segment" << std::endl;
      // Add point on segment to tree
      tree.emplace_back(createNode(nearestPointOnSegment));
      // Make new node point to the nearest neighbor
      tree.back().neighbors.emplace_back(neighbor);
      return true;  // Indicates we connected two trees together by segment
    }
  }

  return false;
}

} // end namespace rrt
