/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/
// /usr/include
#include <iostream>
#include <thread>
#include <cmath>
#include <random>
#include <cstdlib>
#include <utility>

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
  if(otherTree == nullptr) {     // other Tree will set up objects if not nullptr
    setWalls();
    setUpObjects();
  } else {
    objects = otherTree->getObjectAndTransform();
    if(objects.empty()) {
      std::cout << "There are no objects" << std::endl;
    } else {
      std::cout << "Objects copied from other tree" << std::endl;
      objectInMap = otherTree->getObjects();
    }

    walls = otherTree->getWallsAndTransform();
    if(walls.empty()) {
      std::cout << "There are no walls" << std::endl;
    } else {
      std::cout << "Walls copied from other tree" << std::endl;
      objectInMap = otherTree->getObjects();
    }
  }

  vector2f1 startPt;
  bool collision = true;
  while(collision) {
    startPt.x() = get_random(-boundaryWidth_m, boundaryWidth_m);
    startPt.y() = get_random(-boundaryHeight_m, boundaryHeight_m);
    collision = collisionDetection(startPt, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform());
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

void rapidRandomTree::growTreeTowardsRandom() {
  vector2f1 randomPoint, newPoint, pointToKeep;
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
      randomPointCollison = collisionDetection(randomPoint, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform());
    }

    // Find index of closest neighbor
    neighbor = findClosestNeighbor(randomPoint, tree);

    // Check against other tree for direct connection first from nearest neighbor to its nearest neighbor
    randomPoint = connectToOtherTree(randomPoint, neighbor);

    // Grow tree from closest neighbor towards random point by distance epsilon
    newPoint = projectToPointOnLine(tree.at(neighbor).location_m, randomPoint, maxStepDistance_m);

    // Verify no collision from neighbor location to new point location
    int stepSize = (int) (maxStepDistance_m/robotRadius);
    if(stepSize > 0) {
      for (int i = stepSize; i > 0; i--) {
        float distance = maxStepDistance_m / ((float) i);
        auto subPoint = projectToPointOnLine(tree.at(neighbor).location_m, newPoint, distance);

        // Check if robot will collide at subpoint
        if (!collisionDetection(subPoint, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform())) {
          pointToKeep = subPoint;
          robotoCollision = false;
        } else {
          break;    // Will use last point to Keep
        }
      }
    } else {
      pointToKeep = newPoint;
      robotoCollision = collisionDetection(newPoint, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform());
    }
  }

  newPoint = pointToKeep;

  // Set node of neighbor for access
  setConnectingNeighbor(tree.at(neighbor));           // graph will use neighbor location as vertex
  // Make nearest neighbor point to new node.
  tree.at(neighbor).neighbors.emplace_back(tree.size()); // Id of new node being placed in tree
  // Add new point to tree
  tree.emplace_back(createNode(newPoint));
  lastNodeCoordinate = newPoint;                        // last point added to tree, other vertex in graph
}

void rapidRandomTree::growTreeTowardsPoint(vector2f1& setPt) {
  vector2f1 newPoint, pointToKeep;

  // Find closest neighbor
  auto neighbor = findClosestNeighbor(setPt, tree);

  // Check if setPt can connect to nearest neighbor or neighbor segment
  if(connectToNeighborSegment(setPt, neighbor)) {
    return;
  }

  // Get distance from setPt to nearest neighbor
  auto distanceToNeighbor = distance(setPt, tree.at(neighbor).location_m);
  if(distanceToNeighbor < maxStepDistance_m) {
    setConnectingNeighbor(tree.at(neighbor));           // Can be used to connect last start tree point to this neighbor
    lastNodeCoordinate = tree.at(neighbor).location_m;    // The neighbor coordinate to connect to
    reachedGoalPoint = true;
    return;
  }

  // Grow tree from closest neighbor towards point by distance epsilon
  newPoint = projectToPointOnLine(tree.at(neighbor).location_m, setPt, maxStepDistance_m);

  bool robotoCollision = true;
  // Verify no collision from neighbor location to new point location
  int stepSize = (int) (maxStepDistance_m/robotRadius);
  if(stepSize > 0) {
    for (int i = stepSize; i > 0; i--) {
      float distance = maxStepDistance_m / ((float) i);
      auto subPoint = projectToPointOnLine(tree.at(neighbor).location_m, newPoint, distance);

      // Check if robot will collide at subpoint
      if (!collisionDetection(subPoint, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform())) {
        pointToKeep = subPoint;
        robotoCollision = false;
      } else {
        break;    // Will use last point to Keep, or may not be able to extend tree if robotCollision is true
      }
    }
  } else {
    pointToKeep = newPoint;
    robotoCollision = collisionDetection(newPoint, getRobotAndTransform(), getObjectAndTransform(), getWallsAndTransform());
  }

  if(robotoCollision) {
    return;                     // return without adding new point to tree
  }
  newPoint = pointToKeep;


  // Store neighbor connecting to new point
  setConnectingNeighbor(tree.at(neighbor));   // On Graph will connect new point to this neighbor node
  lastNodeCoordinate = newPoint;                 // point of the neighbor node being connected to new point

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

uint64_t rapidRandomTree::findClosestNeighbor(const vector2f1& randomPt, const std::vector<node>& searchTree) {
  if(searchTree.empty()) {
    std::cout << "Tree is empty. This value is invalid" << std::endl;
    exit(0);
  }

  std::vector<float> distances;
  for(const auto& it : searchTree) {
    distances.emplace_back(distance(it.location_m, randomPt));
  }

  // pull the iterator with the smallest distance value
  auto it = std::min_element(distances.begin(), distances.end());
  auto index = std::distance(distances.begin(), it);  // get the index value of the vector at iterator
  return index;
}

vector2f1 rapidRandomTree::projectToPointOnLine(vector2f1& startPt, vector2f1& endPt, float distanceToProject_m) {
  vector2f1 newPoint;
  float x;
  float y;

  // MAG of line segment from start to end
  float dy = endPt.y()-startPt.y();
  float dx = endPt.x()-startPt.x();
  if(dx == 0.f) {                         // Means we are on a vertical line
    x = endPt.x();
    if(dy > 0) {
      y = startPt.y() + distanceToProject_m;
      if(y > endPt.y()) {                 // Do not want to set new point past endpoint
        y = endPt.y();
      }
    } else {
      y = startPt.y() - distanceToProject_m;
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
      x = startPt[0] + distanceToProject_m;
      if(x > endPt.x()) {                 // Do not want to set new point past endpoint
        x = endPt.x();
      }
    } else {
      x = startPt.x() - distanceToProject_m;
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
  double distanceFromEnd = MAG - distanceToProject_m;
  if(distanceFromEnd > 0) {               // goal point not past end point
    double b = startPt[1] - SLOPE*startPt[0];
    // Use dist formula with goal.Y subbed in to get goal.X
    double c = b - startPt[1];
    double A = pow(SLOPE,2) + 1;
    double B = 2*c*SLOPE - 2*startPt[0];
    double C = pow(c,2) + pow(startPt[0],2) - pow(distanceToProject_m,2);
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
  border.clear();
  walls.clear();
  Transform3f tf;
  tf.setIdentity();
  tf.translation().z() = 0;
  tf.rotation().eulerAngles(0, 1, 2)[0] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[1] = 0;
  tf.rotation().eulerAngles(0, 1, 2)[2] = 0;

  // Wall1
  newObject.location_m.x() = boundaryWidth_m + 0.050f;
  newObject.location_m.y() = 0;
  newObject.width = 0.12;
  newObject.height = 2*boundaryHeight_m;
  newObject.objectId = border.size();
  tf.translation().x() = newObject.location_m.x();
  tf.translation().y() = newObject.location_m.y() = 0;
  std::shared_ptr<Boxf> wall1(new Boxf(newObject.width, newObject.height, 0));
  wall = {wall1, tf};
  walls.emplace_back(wall);
  border.emplace_back(newObject);

  // Wall2
  newObject.location_m.x() = -boundaryWidth_m - 0.050f;
  newObject.location_m.y() = 0;
  newObject.objectId = border.size();
  tf.translation().x() = newObject.location_m.x();
  tf.translation().y() = newObject.location_m.y();
  std::shared_ptr<Boxf> wall2(new Boxf(newObject.width, newObject.height, 0));
  wall = {wall2, tf};
  walls.emplace_back(wall);
  border.emplace_back(newObject);

  // Wall3
  newObject.location_m.x() = 0;
  newObject.location_m.y() = -boundaryHeight_m - 0.050f;
  newObject.width = 2*boundaryWidth_m;
  newObject.height = 0.12;
  newObject.objectId = border.size();
  tf.translation().x() = newObject.location_m.x();
  tf.translation().y() = newObject.location_m.y();
  std::shared_ptr<Boxf> wall3(new Boxf(newObject.width, newObject.height, 0));
  wall = {wall3, tf};
  walls.emplace_back(wall);
  border.emplace_back(newObject);

  // Wall4
  newObject.location_m.x() = 0;
  newObject.location_m.y() = boundaryHeight_m + 0.050f;
  newObject.objectId = border.size();
  tf.translation().x() = newObject.location_m.x();
  tf.translation().y() = newObject.location_m.y();
  std::shared_ptr<Boxf> wall4(new Boxf(newObject.width, newObject.height, 0));
  wall = {wall4, tf};
  walls.emplace_back(wall);
  border.emplace_back(newObject);
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

bool rapidRandomTree::collisionDetection(const vector2f1& point,
                                         std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>  robotModel,
                                         std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> objects,
                                         std::vector<std::pair<std::shared_ptr<fcl::Boxf>, fcl::Transform3f>> walls) {
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

    // It is known no other node will be closer than the actual neighbor node
    // So only check if nearestPointOnSegment came out to be the neighbor node location
    if(nearestPointOnSegment == tree.at(neighbor).location_m) {
      return false;                       // Will later check if can directly connect to neighbor
    }

    auto distanceToNearest = distance(nearestPointOnSegment, queryPt);
    if(distanceToNearest <= maxStepDistance_m) {
      // Add point on segment to tree
      tree.emplace_back(createNode(nearestPointOnSegment));
      // Newest node will point to nearest neighbor
      tree.back().neighbors.emplace_back(neighbor);

      // Store connecting node to new point for access
      setConnectingNeighbor(tree.back());             // This is the node the start tree will connect to
      lastNodeCoordinate = nearestPointOnSegment;        // This is the location of the connecting node
      reachedGoalPoint = true;
      return true;  // Indicates we connected two trees together by segment
    }
  }

  return false;
}

vector2f1 rapidRandomTree::connectToOtherTree(vector2f1& point, uint64_t thisNeighbor) {
  vector2f1 neighborLocation_m = tree.at(thisNeighbor).location_m;
  if(otherTree.empty()) {
    return point;
  }

  auto otherTreeNearestNeighbor = findClosestNeighbor(neighborLocation_m, otherTree);
  auto otherNeighbor_m = otherTree.at(otherTreeNearestNeighbor).location_m;

  // Check nearest neighbor in other tree to it nearest neighbors for intersection of current neighbor to point
  for(const auto it : otherTree.at(otherTreeNearestNeighbor).neighbors) {
    auto otherNeighborsNeighbor_m = otherTree.at(it).location_m;
    if(lineIntersection(point, point, neighborLocation_m, otherNeighbor_m, otherNeighborsNeighbor_m, maxStepDistance_m)) {
      // Means the point does intersect a segment on the other tree
      return point;
    }
  }

  return point;
}

bool rapidRandomTree::lineIntersection(vector2f1 &intersectPt, vector2f1 A, vector2f1 B,
                                       vector2f1 C, vector2f1 D, float step_m) {
  /*
    1. a1x + b1y = c1
    2. a2x + b2y = c2

    multiply 1. by b2 and 2 by b1

    1. a1b2x + b1b2y = c1b2
    2. a2b1x + b2b1y = c2b1

    subtract 1 and 2

    (a1b2 – a2b1) x = c1b2 – c2b1

    Solve for x. Do similar operation for y
  */

  float x, y;
  // Line AB represented as a1x + b1y = c1
  float a1 = B.y() - A.y();
  float b1 = A.x() - B.x();
  float c1 = a1*(A.x()) + b1*(A.y());

  // Line CD represented as a2x + b2y = c2
  float a2 = D.y() - C.y();
  float b2 = C.x() - D.x();
  float c2 = a2*(C.x())+ b2*(C.y());

  float determinant = a1*b2 - a2*b1;
  if(determinant == 0) {
    // Lines are parallel
    return false;
  } else {
    x = (b2*c1 - b1*c2)/determinant;
    y = (a1*c2 - a2*c1)/determinant;
  }

  if(x < std::min(A.x(), B.x()) || x > std::max(A.x(), B.x())) {
    return false;
  }

  if(x < std::min(C.x(), D.x()) || x > std::max(C.x(), D.x())) {
    return false;
  }

  if(y < std::min(A.y(), B.y()) || y > std::max(A.y(), B.y())) {
    return false;
  }

  if(y < std::min(C.y(), D.y()) || y > std::max(C.y(), D.y())) {
    return false;
  }

  vector2f1 temp;
  temp.x() = x;
  temp.y() = y;
  auto dist = distance(temp, B);
  if(dist > step_m) {
    return false;
  }

  // ptA2 is the neigbor node location of this object's tree
  intersectPt = projectToPointOnLine(temp, B, dist/2);

  return true;
}

} // end namespace rrt
