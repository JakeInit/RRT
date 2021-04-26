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
#include <cstdlib>
#include <utility>
#include <boost/random.hpp>

// local include
#include "rapidRandomTree.h"
#include "jsonParser.h"
#include "timerEvent.h"

namespace rrt {

using namespace fcl;

rrt::system::jsonParser* configReader = nullptr;

node::node() {
  goalNode = false;
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

// Constructor
rapidRandomTree::rapidRandomTree(const std::string& treeName_, float robotRadius_, rapidRandomTree* otherTree, bool goalTree_) {
  treeName = treeName_;
  robotRadius = robotRadius_;

  configReader = rrt::system::jsonParser::getInstance();
  maxStepDistance_m = configReader->parametersForSystem.stepDistance_m;
  boundaryWidth_m = configReader->parametersForSystem.boundaryWidth_m;
  boundaryHeight_m = configReader->parametersForSystem.boundaryHeight_m;
  maxObjectSize_m = configReader->parametersForSystem.maxObjectSize_m;
  numberOfObjects = configReader->parametersForSystem.maxObjects;

  setUpRobotModel();
  setWalls();
  if(otherTree == nullptr) {     // other Tree will set up objects if not nullptr
    setUpObjects();
  } else {
    objects = otherTree->getObjectAndTransform();
    if(objects.empty()) {
      std::cout << "There are no objects" << std::endl;
    } else {
      std::cout << "Objects copied from other tree" << std::endl;
      objectInMap = otherTree->getObjects();
    }
  }

  vector2f1 startPt;
  bool collision = true;
  while(collision) {
    startPt.x() = get_random(-boundaryWidth_m, boundaryWidth_m);
    startPt.y() = get_random(-boundaryHeight_m, boundaryHeight_m);
    collision = collisionDetection(startPt);
  }

  initPoint = startPt ;
  placeRobotInMap();
  std::cout << "Creating new tree " << treeName << " at point {x, y} = {" <<
    startPt.x() << ", " << startPt.y() << "}" << std::endl;

  tree.clear();
  auto newNode = createNode(startPt);
  if(goalTree_) {
    newNode.goalNode = true;
  }
  tree.emplace_back(newNode);
  reachedGoalPoint = false;
}

// De-Constructor
rapidRandomTree::~rapidRandomTree() {
  std::cout << "De-constructing tree " << treeName << std::endl;
  if(configReader != nullptr) {
    rrt::system::jsonParser::deleteInstance();
    configReader = nullptr;
  }
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
      randomPoint.x() = get_random(-boundaryWidth_m + robotRadius/2, boundaryWidth_m - robotRadius/2);
      randomPoint.y() = get_random(-boundaryHeight_m + robotRadius/2, boundaryHeight_m - robotRadius/2);

      // Check collision detector of random point
      randomPointCollison = collisionDetection(randomPoint);
    }

    // Find index of closest neighbor
    neighbor = findClosestNeighbor(randomPoint);

    // Grow tree from closest neighbor towards random point by distance epsilon
    newPoint = projectToPointOnLine(tree.at(neighbor).location_m, randomPoint);

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

  // Check if setPt can connect to nearest neighbor or neighbor segment
  if(connectToNeighborSegment(setPt, neighbor)) {
    // Store node of connecting neighbor to new point
    setConnectingNeighbor(tree.at(neighbor));
    lastNodeCoordinate = newPoint;
    reachedGoalPoint = true;
    return;
  }

  // Grow tree from closest neighbor towards point by distance epsilon
  newPoint = projectToPointOnLine(tree.at(neighbor).location_m, setPt);

  // Check collision detector
  if(collisionDetection(newPoint)) {
    return;                               // return without adding new point to tree
  }

  // Store node of connecting neighbor for get function
  setConnectingNeighbor(tree.at(neighbor));
  lastNodeCoordinate = newPoint;

  // Add new point to tree
  tree.emplace_back(createNode(newPoint));
  // Make new node point to the nearest neighbor.
  // This will allow the start tree to point to the connecting node
  //  in the goal tree and then find the goal point
  tree.back().neighbors.emplace_back(neighbor);
}

node rapidRandomTree::createNode(vector2f1 newPt) {
  node newNode;
  newNode.location_m.x() = newPt.x();
  newNode.location_m.y() = newPt.y();
  newNode.id = (uint64_t) tree.size();
  return newNode;
}

uint64_t rapidRandomTree::findClosestNeighbor(const vector2f1& randomPt) {
  if(tree.empty()) {
    std::cout << "Tree is empty. This value is invalid" << std::endl;
    exit(0);
  }

  std::vector<float> distances;
  for(const auto& it : tree) {
    distances.emplace_back(distance(it.location_m, randomPt));
  }

  // pull the iterator with the smallest distance value
  auto it = std::min_element(distances.begin(), distances.end());
  auto index = std::distance(distances.begin(), it);  // get the index value of the vector at iterator
  return index;
}

vector2f1 rapidRandomTree::projectToPointOnLine(vector2f1& startPt, vector2f1& endPt) {
  vector2f1 newPoint;
  float x;
  float y;

  // MAG of line segment from start to end
  float dy = endPt.y()-startPt.y();
  float dx = endPt.x()-startPt.x();
  if(dx == 0.f) {                         // Means we are on a vertical line
    x = endPt.x();
    if(dy > 0) {
      y = startPt.y() + maxStepDistance_m;
      if(y > endPt.y()) {                 // Do not want to set new point past endpoint
        y = endPt.y();
      }
    } else {
      y = startPt.y() - maxStepDistance_m;
      if(y < endPt.y()) {                 // Do not want to set new point past endpoint
        y = endPt.y();
      }
    }
    newPoint.x() = x;
    newPoint.y() = y;
    return newPoint;
  } else if(dy == 0.f) {                  // Means we are on a horizontal line
    y = endPt.y();
    if(dx > 0) {
      x = startPt[0] + maxStepDistance_m;
      if(x > endPt.x()) {                 // Do not want to set new point past endpoint
        x = endPt.x();
      }
    } else {
      x = startPt.x() - maxStepDistance_m;
      if(x < endPt.x()) {                 // Do not want to set new point past endpoint
        x = endPt.x();
      }
    }
    newPoint.x() = x;
    newPoint.y() = y;
    return newPoint;
  }

  float MAG = distance(startPt, endPt);
  float SLOPE = dy/dx;

  // determine if epsilon goes past Goal point
  double distanceFromEnd = MAG - maxStepDistance_m;
  if(distanceFromEnd > 0) {               // goal point not past end point
    double b = startPt[1] - SLOPE*startPt[0];
    // Use dist formula with goal.Y subbed in to get goal.X
    double c = b - startPt[1];
    double A = pow(SLOPE,2) + 1;
    double B = 2*c*SLOPE - 2*startPt[0];
    double C = pow(c,2) + pow(startPt[0],2) - pow(maxStepDistance_m,2);
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
  std::random_device rd;
  std::uniform_real_distribution<> dist(lowerBound, upperBound);
  return (float) dist(rd);
}

void rapidRandomTree::setUpRobotModel() {
  robotModel.first = std::make_unique<Boxf>(robotRadius, robotRadius, 0);
  Transform3f tfRobot;
  tfRobot.setIdentity();
  tfRobot.translation().x() = 0;
  tfRobot.translation().y() = 0;
  tfRobot.translation().z() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).x() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).y() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).z() = 0;
  robotModel.second = tfRobot;
}

void rapidRandomTree::setWalls() {
  std::pair<std::shared_ptr<Boxf>, Transform3f> wall;
  objectNode newObject;
  Transform3f tf;
  tf.setIdentity();
  tf.translation().z() = 0;
  tf.rotation().eulerAngles(0, 1, 2)[0] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[1] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[2] = 0;

  tf.translation().x() = boundaryWidth_m + 0.050f;
  tf.translation().y() = 0;
  std::shared_ptr<Boxf> wall1(new Boxf(0.10, boundaryHeight_m, 0));
  wall = {wall1, tf};
  walls.emplace_back(wall);

  tf.translation().x() = -boundaryWidth_m - 0.050f;
  tf.translation().y() = 0;
  std::shared_ptr<Boxf> wall2(new Boxf(0.10, boundaryHeight_m, 0));
  wall = {wall2, tf};
  walls.emplace_back(wall);

  tf.translation().x() = 0;
  tf.translation().y() = -boundaryHeight_m - 0.050f;
  std::shared_ptr<Boxf> wall3(new Boxf(boundaryWidth_m, 0.10, 0));
  wall = {wall3, tf};
  walls.emplace_back(wall);

  tf.translation().x() = 0;
  tf.translation().y() = boundaryHeight_m + 0.050f;
  std::shared_ptr<Boxf> wall4(new Boxf(boundaryWidth_m, 0.10, 0));
  wall = {wall4, tf};
  walls.emplace_back(wall);
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

  bool collision;
  numberOfObjects = (uint64_t) get_random(0.f, (float) numberOfObjects);
  std::cout << "Number of objects = " << numberOfObjects << std::endl;
  for(int i = 0; i < numberOfObjects; i++) {
    newObject.height = get_random(0.100f, maxObjectSize_m);
    newObject.width = newObject.height;

    collision = true;
    while(collision) {
      newObject.location_m.x() = get_random(-boundaryWidth_m, boundaryWidth_m);
      newObject.location_m.y() = get_random(-boundaryHeight_m, boundaryHeight_m);
      collision = newObstacleCollisionDetection(newObject);
    }

    tf.translation().x() = newObject.location_m.x();
    tf.translation().y() = newObject.location_m.y();
    std::shared_ptr<Boxf> box(new Boxf(newObject.width, newObject.height, 0));
    object = {box, tf};
    objects.emplace_back(object);

    newObject.objectId = i;
    objectInMap.emplace_back(newObject);
  }
}

bool rapidRandomTree::collisionDetection(const vector2f1& point) {
  CollisionRequestf request;
  CollisionResultf result;

  robotModel.second.translation().x() = point.x();  // This places the robot at the point inserted
  robotModel.second.translation().y() = point.y();  // This will tell if point is even viable for the robot to exist at
  CollisionObjectf object1(robotModel.first, robotModel.second);
  if(!objects.empty()) {
    for (const auto &it : objects) {
      CollisionObjectf object2(it.first, it.second);
      collide(&object1, &object2, request, result);
      if (result.isCollision()) {
        return true;                        // Point is not viable if collision is detected with any object
      }
    }
  }

  if(walls.empty()) {
    return false;
  }

  // Check walls as well
  for(const auto& it : walls) {
    CollisionObjectf object2(it.first, it.second);
    collide(&object1, &object2, request, result);
    if(result.isCollision()) {
      return true;                        // Point is not viable if collision is detected with wall
    }
  }

  return false;
}

bool rapidRandomTree::newObstacleCollisionDetection(objectNode &newObject) {
  Transform3f tfNewObject;
  tfNewObject.setIdentity();
  tfNewObject.translation().x() = newObject.location_m.x();
  tfNewObject.translation().y() = newObject.location_m.y();
  tfNewObject.translation().z() = 0;
  tfNewObject.rotation().eulerAngles(0, 1, 2).x() = 0;
  tfNewObject.rotation().eulerAngles(0, 1, 2).y() = 0;
  tfNewObject.rotation().eulerAngles(0, 1, 2).z() = 0;

  std::shared_ptr<Boxf> box(new Boxf(newObject.width, newObject.height, 0));

  CollisionRequestf request;
  CollisionResultf result;
  CollisionObjectf object1(box, tfNewObject);
  if(!objects.empty()) {
    for (const auto &it : objects) {
      CollisionObjectf object2(it.first, it.second);
      collide(&object1, &object2, request, result);
      if (result.isCollision()) {
        return true;                        // Point is not viable if collision is detected with any object
      }
    }
  }

  if(walls.empty()) {
    return false;
  }

  // Check walls as well
  for(const auto& it : walls) {
    CollisionObjectf object2(it.first, it.second);
    collide(&object1, &object2, request, result);
    if(result.isCollision()) {
      return true;                        // Point is not viable if collision is detected with wall
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

  for(auto it : neighborNode.neighbors) { // Check all segments of nodes connecting to neighbor
    auto nearestPointOnSegment = closestPointOnSegment(neighborNode.location_m, tree.at(it).location_m, queryPt);
    auto distanceToNearest = distance(nearestPointOnSegment, queryPt);
    if(distanceToNearest <= maxStepDistance_m) {
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
