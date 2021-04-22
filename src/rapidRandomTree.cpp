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
#include "fcl/math/detail/project.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/math/sampler/sampler_r.h"
#include "fcl/math/sampler/sampler_se2.h"
#include "fcl/math/sampler/sampler_se2_disk.h"
#include "fcl/math/sampler/sampler_se3_euler.h"
#include "fcl/math/sampler/sampler_se3_euler_ball.h"
#include "fcl/math/sampler/sampler_se3_quat.h"
#include "fcl/math/sampler/sampler_se3_quat_ball.h"
#include "fcl/math/geometry.h"

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"

namespace rrt {

using namespace fcl;
typedef BVHModel<OBBRSSf> Model;

node::node() {
  location_m.setZero();
  id = 0;
  neighbors.clear();
}

// Constructor
rapidRandomTree::rapidRandomTree(const std::string& treeName_, float robotRadius_) {
  treeName = treeName_;
  robotRadius = robotRadius_;
  setUpRobotModel();
  setUpObjects();
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
  // set mesh triangles and vertice indices
  std::vector<Vector3f> vertices;
  std::vector<Triangle> triangles;
  // code to set the vertices and triangles
  Triangle robot;
  robot.set(1, 1, 1);

  // BVHModel is a template class for mesh geometry, for default OBBRSS template
  // is used
  std::shared_ptr<Model> geom = std::make_shared<Model>();
  // add the mesh data into the BVHModel structure
  geom->beginModel();
  geom->addSubModel(vertices, triangles);
  geom->endModel();

  // R and T are the rotation matrix and translation vector
  Matrix3f R;
  Vector3f T;

  // code for setting R and T goes below this line

  // transform is configured according to R and T
  Transform3f pose = Transform3f::Identity();
  pose.linear() = R;
  pose.translation() = T;

  //geom and tf are the geometry and the transform of the object
//  std::shared_ptr<BVHModel<OBBRSSf>> geom = ...
//  Transform3f tf = ...
//
//  //Combine them together
//  CollisionObjectf* obj = new CollisionObjectf(geom, tf);
}

void rapidRandomTree::setUpObjects() {

}

bool rapidRandomTree::collisionDetection(const vector2f1& point) {
  // Given two objects o1 and o2
//  CollisionObjectf* o1 = ...
//  CollisionObjectf* o2 = ...

  // set the collision request structure, here we just use the default setting
//  CollisionRequest request;

  // result will be returned via the collision result structure
//  CollisionResult result;

  // perform collision test
//  collide(o1, o2, request, result);
  return false;
}

} // end namespace rrt
