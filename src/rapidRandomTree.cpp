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

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"

namespace rrt {

node::node() {
  coordinate.x_m = 0;
  coordinate.y_m = 0;
  id = 0;
  neighbors.clear();
}

// Constructor
rapidRandomTree::rapidRandomTree(const std::string& treeName_) {
  treeName = treeName_;
  float x_m = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
  float y_m = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);
  std::cout << "Creating new tree " << treeName << " at point {x, y} = {" <<
    x_m << ", " << y_m << "}" << std::endl;

  tree.clear();
  tree.emplace_back(createNode(x_m, y_m));
  treeIsValid = true;
}

// De-Constructor
rapidRandomTree::~rapidRandomTree() {
  std::cout << "De-constructing tree " << treeName << std::endl;
}

std::pair<float, float> rapidRandomTree::growTreeTowardsRandom() {
  // Create a random point
  float x_m = get_random(-BOUNDARYWIDTH, BOUNDARYWIDTH);
  float y_m = get_random(-BOUNDAYHEIGHT, BOUNDAYHEIGHT);

  // Find closest neighbor
  auto neighbor = findClosestNeighbor(x_m, y_m);

  // Grow tree from closest neighbor towards random point by distance epsilon
  // Add new point to tree
  // returns xy-coordinates of new point
  return {};
}

void rapidRandomTree::growTreeTowardsPoint(float x_m, float y_m) {

}

node rapidRandomTree::createNode(float x_m, float y_m) {
  node newNode;
  newNode.coordinate.x_m = x_m;
  newNode.coordinate.y_m = y_m;
  newNode.id = (uint64_t) tree.size();
  return newNode;
}

uint64_t rapidRandomTree::findClosestNeighbor(float x_m, float y_m) {
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
    distances.emplace_back(distance(it.coordinate.x_m, x_m, it.coordinate.y_m, y_m));
  }

  auto it = std::min_element(distances.begin(), distances.end());
  return std::distance(distances.begin(), it);
}

float rapidRandomTree::distance(float x1_m, float y1_m, float x2_m, float y2_m) {
  float dx = x1_m - x2_m;
  float dy = y1_m - y2_m;
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

} // end namespace rrt
