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

using namespace mainspace;
std::unique_ptr<rrt::rapidRandomTree> qInit = nullptr;
std::unique_ptr<rrt::rapidRandomTree> qGoal = nullptr;
uint16_t counter = 0;

int main(int argc, char** argv) {
  running = true;
  signal(SIGINT, mainspace::signalHandler);

//  // create the window
//  sf::RenderWindow window(sf::VideoMode(800, 600), "My window");
//
//  sf::RectangleShape line;
//  line.setSize(sf::Vector2f(200, 3));
//
//  // create an array of 3 vertices that define a triangle primitive
//  sf::VertexArray border(sf::LineStrip, 3);
//
//  // define the position of the triangle's points
//  border[0].position = sf::Vector2f(10.f, 10.f);
//  border[1].position = sf::Vector2f(100.f, 10.f);
//  border[2].position = sf::Vector2f(100.f, 100.f);
//
//  // define the color of the triangle's points
//  border[0].color = sf::Color::Red;
//  border[1].color = sf::Color::Red;
//  border[2].color = sf::Color::Red;
//
//  std::cout << "While window is open" << std::endl;
//  while(window.isOpen()) {
//    // check all the window's events that were triggered since the last iteration of the loop
//    sf::Event event{};
//    while (window.pollEvent(event)) {
//      // "close requested" event: we close the window
//      if (event.type == sf::Event::Closed)
//        window.close();
//    }
//
//    // clear the window with black color
//    window.clear(sf::Color::Black);
//
//    // draw everything here...
//    window.draw(line);
//    window.draw(border);
//
//    // end the current frame
//    window.display();
//  }

  rrt::system::timerEvent timer;
  const float loopTime_ms = 10;

  qInit = std::make_unique<rrt::rapidRandomTree>("StartPoint", ROBOTRADIUS);
  qGoal = std::make_unique<rrt::rapidRandomTree>("GoalPoint", ROBOTRADIUS);

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
    qGoal->growTreeTowardsPoint(newStartTreePoint);
    if(qGoal->goalReached()) {
      std::cout << "Trees are connected. Path from start to goal is possible" << std::endl;
      double endTime = rrt::system::timerEvent::getRunTime_ms();
      std::cout << "Time to connect trees = " << endTime - startTime << "ms" << std::endl;
      std::cout << "Number of attempts to grow trees = " << counter << std::endl;
      exit(0);
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
  std::cout << "Shutting down the application." << std::endl;
}

void mainspace::signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received. Setting running to false" << std::endl;
  running = false;
}
