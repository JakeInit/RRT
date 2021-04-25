/*
  Author:   Jacob Morgan
  Date:     02/15/21
  Company:  Mechaspin
  Extra:
*/

// /usr/include
#include <iostream>
#include <thread>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <memory>
#include <SFML/Graphics.hpp>

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"
#include "main.h"

#define ROBOTRADIUS 1.0
#define ATTEMPTS    1000
#define WINDOWDIM   800.f
#define LOOPTIME_MS 100

using namespace mainspace;
std::unique_ptr<rrt::rapidRandomTree> qInit = nullptr;
std::unique_ptr<rrt::rapidRandomTree> qGoal = nullptr;
std::unique_ptr<sf::RenderWindow> window = nullptr;
uint16_t counter = 0;

float windowResolution;

void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2);
void placeObjectInMap(rrt::objectNode objectInMap);
rrt::vector2f1 convertPointToWindow(rrt::vector2f1 point);

int main(int argc, char** argv) {
  running = true;
  signal(SIGINT, mainspace::signalHandler);

  // create the window
  window = std::make_unique<sf::RenderWindow>(sf::VideoMode(WINDOWDIM, WINDOWDIM), "Dual RRT");
  windowResolution = 2*BOUNDARYWIDTH/WINDOWDIM;
  std::cout << "Window Resolution = " << windowResolution << std::endl;

  rrt::system::timerEvent timer;
  const float loopTime_ms = LOOPTIME_MS;

  qInit = std::make_unique<rrt::rapidRandomTree>("StartPoint", ROBOTRADIUS);
  qGoal = std::make_unique<rrt::rapidRandomTree>("GoalPoint", ROBOTRADIUS);
  auto objects = qGoal->getObjects();
  if(!objects.empty()) {
    for(auto it : objects) {
      placeObjectInMap(it);
    }
  }

  std::cout << "Linear distance between start and end = " << rrt::rapidRandomTree::distance(qInit->getTreeStart(),
                                                                                    qGoal->getTreeStart()) << std::endl;

  double startTime = rrt::system::timerEvent::getRunTime_ms();
  std::cout << "Entering the Running Loop" << std::endl;
  while(running) {
    if(timer.timerDone()) {
      timer.runNonBlockingTimer_ms(std::floor(loopTime_ms));
    } else {
      timer.waitForTimerToFinish();
      continue;
    }

    // Grow the trees until they connect
    rrt::vector2f1 newStartTreePoint = qInit->growTreeTowardsRandom();
    rrt::vector2f1 neighborNode = qInit->getConnectingNeighbor().location_m;
    updateWindow(newStartTreePoint, neighborNode);

    qGoal->growTreeTowardsPoint(newStartTreePoint);
    if(!qGoal->goalReached()) {
      rrt::vector2f1 lastPoint = qGoal->getCoordinateOfLastNode();
      neighborNode = qGoal->getConnectingNeighbor().location_m;
      updateWindow(lastPoint, neighborNode);
    } else {
      updateWindow(newStartTreePoint, qGoal->getConnectingNeighbor().location_m);
    }


    if(qGoal->goalReached()) {
      std::cout << "Trees are connected. Path from start to goal is possible" << std::endl;
      double endTime = rrt::system::timerEvent::getRunTime_ms();
      std::cout << "Time to connect trees = " << endTime - startTime << "ms" << std::endl;
      std::cout << "Number of attempts to grow trees = " << counter << std::endl;
      // This means the last point added to start tree connects to goal tree
      // Need the id of the goal tree for the start tree to connect to it
      // Then combine the trees into 1 vector
      //  1.) add the size of the start vector to the node id in the goal tree
      //  2.) add the size of the start vector to the node ids in the neighbors vector of each node in the goal tree
      //  3.) add the size of the start vector to the connecting node of the goal tree
      //  4.) add the connect node of the goal tree to the last point created in the start tree in the neighbors list
      auto startTree = qInit->getTree();
      auto goalTree = qGoal->getTree();
      auto connectingGoalNode = qGoal->getConnectingNeighbor();

      for(auto it : goalTree) {           // Iterate through each node in the goal tree
        it.id += startTree.size();
        for(auto it2 : it.neighbors) {    // Iterate through every neighbor in each node of goal tree
          it2 += startTree.size();
        }
      }

      connectingGoalNode.id += startTree.size();

      // Add index of connecting goal node as a neighbor in the nearest node of the startTree
      startTree.at(qInit->getIdOfLastPoint()).neighbors.emplace_back(connectingGoalNode.id);

      // Create the map of possible paths for the robot in the environment
      std::vector<rrt::node> pathMap = startTree;
      pathMap.insert(pathMap.end(), goalTree.begin(), goalTree.end());
      running = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } else {
      counter++;
    }

    if(counter >= ATTEMPTS) {
      running = false;
      std::cout << "No viable path found in " << ATTEMPTS << " to grow trees." << std::endl;
      double endTime = rrt::system::timerEvent::getRunTime_ms();
      std::cout << "Time ran = " << endTime - startTime << "ms" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
  } // end run

  while(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }
  }

  shutdown();
  return 0;
}

void mainspace::shutdown() {
  if(qInit != nullptr) {
    std::cout << "Removing qInit" << std::endl;
    qInit.reset();
  }

  if(qGoal != nullptr) {
    std::cout << "Removing qGoal" << std::endl;
    qGoal.reset();
  }

  if(window != nullptr) {
    std::cout << "Closing Window" << std::endl;
    if(window->isOpen())
      window->close();

    window.reset();
  }
  std::cout << "Shutting down the application." << std::endl;
}

void mainspace::signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received. Setting running to false" << std::endl;
  running = false;
  std::cout << "\nClose Visualizer Window to finish closing down the application" << std::endl;
}

void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2) {
    // create an array of 3 vertices that define a triangle primitive
  sf::VertexArray border1(sf::LineStrip, 2);
  pt1 = convertPointToWindow(pt1);
  pt2 = convertPointToWindow(pt2);

  // define the position of the triangle's points
  border1[0].position = sf::Vector2f(pt1.x(), pt1.y());
  border1[1].position = sf::Vector2f(pt2.x(), pt2.y());

  // define the color of the triangle's points
  border1[0].color = sf::Color::Red;
  border1[1].color = sf::Color::Red;

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

//    // clear the window with black color
//    static bool windowCleared = false;
//    if(!windowCleared) {
//      window->clear(sf::Color::Black);
//      windowCleared = true;
//    }

    // draw everything here...
//    window->draw(line);
    window->draw(border1);

    // end the current frame
    window->display();
  }
}

void placeObjectInMap(rrt::objectNode objectInMap) {
  sf::RectangleShape object(sf::Vector2f(0, 0));
  object.setSize(sf::Vector2f(objectInMap.width/windowResolution,
                              objectInMap.height/windowResolution));

  object.setOrigin(objectInMap.width/(2*windowResolution), objectInMap.height/(2*windowResolution));

  auto convertedPoint = convertPointToWindow(objectInMap.location_m);
  object.setPosition(sf::Vector2f(convertedPoint.x(), convertedPoint.y()));

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

    // draw everything here...
    std::cout << "Drawing Object" << std::endl;
    window->draw(object);

    // end the current frame
    window->display();
  }
}

rrt::vector2f1 convertPointToWindow(rrt::vector2f1 point) {
  rrt::vector2f1 convertedPoint;
  convertedPoint.x() = point.x()/windowResolution;
  convertedPoint.x() += (WINDOWDIM/2);
  convertedPoint.y() = point.y()/windowResolution;
  convertedPoint.y() = (WINDOWDIM/2) - convertedPoint.y();
  return convertedPoint;
}
