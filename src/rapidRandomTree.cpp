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

// fcl includes
//#include "fcl/math/detail/project.h"
//#include "fcl/geometry/bvh/BVH_model.h"
//#include "fcl/math/sampler/sampler_r.h"
//#include "fcl/math/sampler/sampler_se2.h"
//#include "fcl/math/sampler/sampler_se2_disk.h"
//#include "fcl/math/sampler/sampler_se3_euler.h"
//#include "fcl/math/sampler/sampler_se3_euler_ball.h"
//#include "fcl/math/sampler/sampler_se3_quat.h"
//#include "fcl/math/sampler/sampler_se3_quat_ball.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/math/geometry.h"

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"

namespace rrt {

using namespace fcl;

node::node() {
  location_m.setZero();
  id = 0;
  neighbors.clear();
}

std::unique_ptr<Boxf> robotModel;
std::vector<std::pair<Boxf, Transform3f>> objects;

// Constructor
rapidRandomTree::rapidRandomTree(const std::string& treeName_, float robotRadius_) {
  treeName = treeName_;
  robotRadius = robotRadius_;
  setUpRobotModel();
//  setUpObjects();
  vector2f1 startPt;
  startPt.x() = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
  startPt.y() = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);
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
  bool randomPointCollison = true;
  bool robotoCollision = true;

  while(robotoCollision) {
    // Find new point until random point does not collide
    while (randomPointCollison) {
      // Create a random point
      randomPoint.x() = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
      randomPoint.y() = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);

      // Check collision detector of random point
      randomPointCollison = collisionDetection(randomPoint);
    }

    // Find closest neighbor
    auto neighbor = findClosestNeighbor(randomPoint);

    // Grow tree from closest neighbor towards random point by distance epsilon
    newPoint = findPointOnLine(tree.at(neighbor).location_m, randomPoint, true);

    // Check collision detector of new point
    robotoCollision = collisionDetection(newPoint);
  }

  // Add new point to tree
  tree.emplace_back(createNode(newPoint));

  // returns xy-coordinates of new point
  return newPoint;
}

void rapidRandomTree::growTreeTowardsPoint(vector2f1& setPt) {
  vector2f1 newPoint;

  // Find closest neighbor
  auto neighbor = findClosestNeighbor(setPt);

  // Grow tree from closest neighbor towards point by distance epsilon
  newPoint = findPointOnLine(tree.at(neighbor).location_m, setPt, false);

  // Check collision detector
  if(collisionDetection(newPoint)) {
    return;                               // return without adding new point to tree
  }

  // Add new point to tree
  tree.emplace_back(createNode(newPoint));
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
    treeIsValid = false;
    return 0;
  }

  if(tree.size() == 1) {
    return 0;
  }

  std::vector<float> distances;
  for(const auto& it : tree) {
    distances.emplace_back(distance(it.location_m, randomPt));
  }

  auto it = std::min_element(distances.begin(), distances.end());
  return std::distance(distances.begin(), it);
}

vector2f1 rapidRandomTree::findPointOnLine(vector2f1& startPt, vector2f1& endPt, bool toRandom) {
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
  robotModel = std::make_unique<Boxf>(1.0, 1.0, 0);
}

void rapidRandomTree::setUpObjects() {
  std::pair<Boxf, Transform3f> object;
  Transform3f tf;

  tf.translation().x() = 4.875;
  tf.translation().y() = 4.875;
  object = {Boxf(0.25, 0.25, 0), tf};
  objects.emplace_back(object);

  tf.translation().x() = 0.000;
  tf.translation().y() = 3.000;
  object = {Boxf(0.50, 0.50, 0), tf};
  objects.emplace_back(object);

  tf.translation().x() = -2.550;
  tf.translation().y() = -3.000;
  object = {Boxf(0.50, 0.50, 0), tf};
  objects.emplace_back(object);
}

bool rapidRandomTree::collisionDetection(const vector2f1& point) {
  Transform3f tfRobot;
  tfRobot.setIdentity();
  tfRobot.translation().x() = point.x();  // This places the robot at the point inserted
  tfRobot.translation().y() = point.y();  // This will tell if point is even viable for the robot to exist at
  tfRobot.translation().z() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).x() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).y() = 0;
  tfRobot.rotation().eulerAngles(0, 1, 2).z() = 0;

  CollisionRequestf request;
  CollisionResultf result;
  for(auto it : objects) {
    collide(robotModel.get(), tfRobot, &it.first, it.second, request, result);
    if(result.isCollision()) {
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

  return tree.begin()->location_m;
}

} // end namespace rrt
