/*
  Author:   Jacob Morgan
  Date:     02/15/21
  Company:  Mechaspin
  Extra:
*/

// /usr/include
#include <iostream>
#include <fstream>
#include <thread>
#include <csignal>
#include <cmath>
#include <memory>
#include <vector>
#include <SFML/Graphics.hpp>

// local include
#include "rapidRandomTree.h"
#include "jsonParser.h"
#include "timerEvent.h"
#include "main.h"

using namespace mainspace;
std::unique_ptr<rrt::rapidRandomTree> qInit = nullptr;
std::unique_ptr<rrt::rapidRandomTree> qGoal = nullptr;
std::unique_ptr<sf::RenderWindow> window = nullptr;

rrt::rapidRandomTree* randomGrower;       // used to easily swap trees for growing
rrt::rapidRandomTree* controlledGrower;

void watchWindow();
void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2, sf::Color color);

int main(int argc, char** argv) {
  rrt::system::timerEvent timer;
  initialize();
  std::cout << "Entering the Running Loop" << std::endl;

  randomGrower = qInit.get();
  controlledGrower = qGoal.get();
  while(running) {
    if(timer.timerDone()) {
      timer.runNonBlockingTimer_ms(std::floor(loopTime_ms));
    } else {
      timer.waitForTimerToFinish();
      continue;
    }

    // Grow the trees until they connect
    if(!controlledGrower->goalReached()) {
      randomGrower->growTreeTowardsRandom();
      rrt::vector2f1 newTreePoint = randomGrower->getCoordinateOfLastNode();
      rrt::vector2f1 neighborNode = randomGrower->getConnectingNeighbor().location_m; // location of last neighbor
      updateWindow(newTreePoint, neighborNode, sf::Color::Red);                       // Connect the 2 points

      // Grow tree towards other tree
      controlledGrower->growTreeTowardsPoint(newTreePoint);
      if (!controlledGrower->goalReached()) {
        rrt::vector2f1 lastPoint = controlledGrower->getCoordinateOfLastNode();  // location of newest point in tree
        neighborNode = controlledGrower->getConnectingNeighbor().location_m;     // location of last neighbor
        updateWindow(lastPoint, neighborNode, sf::Color::Red);                  // Connect the 2 points
        incrementCounter();

        // Swap the trees
        auto temp = randomGrower;
        randomGrower = controlledGrower;
        controlledGrower = temp;
      } else {                                                        // new point from start connects to goal
        updateWindow(newTreePoint, controlledGrower->getConnectingNeighbor().location_m, sf::Color::Red);
      }
    } else if (pathMap.empty()) {                                     // merge the two RRTs
      mergeTrees();
    } else if (!pathCreated) {                                        // Determine path from start to goal
      aStar();
      if(pathSmootherOn) {
        std::cout << std::endl << "Running Greedy Path Smoother" << std::endl;
        smoothPath();
      }

      std::ofstream myfile;                                           // Write path to txt file
      std::string filePath = PROJECTDIR;
      std::string fileToOpen = filePath + "poses/poses.txt";

      std::ifstream f(fileToOpen.c_str());
      if(f.good()) {
        if(std::remove(fileToOpen.c_str())) {
          std::cout << "Issues deleting file poses.txt" << std::endl;
        } else {
          std::cout << "Removed old file poses.txt" << std::endl;
        }
      }

      myfile.open(fileToOpen.c_str());
      float bearing_rad = 0.f;
      auto startIterator = pathToGoal_m.begin();
      uint64_t pathCounter = 1;
      for(auto& it : pathToGoal_m) {
        if(pathCounter < pathToGoal_m.size()) {
          bearing_rad = getBearing_rad(it, *(startIterator + pathCounter));
        }
        myfile << it.x() << " " << it.y() << " " << bearing_rad << "\n";
        pathCounter++;
      }
      myfile.close();

    } else {                                                          // Visualize new path
      static uint64_t index = 0;
      auto pt1 = pathToGoal_m.at(index);
      auto pt2 = pathToGoal_m.at(index + 1);
      updateWindow(pt1, pt2, sf::Color::Green);

      if(index == pathToGoal_m.size() - 2) {
        running = false;
      } else {
        index++;
      }
    }
  } // end run

  std::cout << std::endl << "Exited the running loop. Close visualizer to terminate application" << std::endl;

  shutdown();
  return 0;
}

void mainspace::initialize() {
  running = true;
  pathCreated = false;
  signal(SIGINT, mainspace::signalHandler);

  configReader = rrt::system::jsonParser::getInstance();
  windowHeight_pix = configReader->parametersForSystem.visualizerHeight_pix;
  windowWidth_pix = configReader->parametersForSystem.visualizerWidth_pix;
  robotHeight_m = configReader->parametersForRobot.dims.height_m;
  robotWidth_m = configReader->parametersForRobot.dims.width_m;

  auto boundaryWidth_m = configReader->parametersForSystem.boundaryWidth_m;
  auto boundaryHeight_m = configReader->parametersForSystem.boundaryHeight_m;
  boundarySize_m = boundaryHeight_m;
  if(boundaryHeight_m > boundaryWidth_m) {    // Must use smallest boundary value
    boundarySize_m = boundaryWidth_m;
  }

  maxNodes = configReader->parametersForSystem.maxNodes;
  pathSmootherOn = configReader->parametersForSystem.pathSmootherOn;

  float loopFrequency_hz = configReader->parametersForSystem.loop_frequency_Hz;
  loopTime_ms = 1000.0f / loopFrequency_hz;

  // create the window
  window = std::make_unique<sf::RenderWindow>(sf::VideoMode(windowWidth_pix, windowHeight_pix), "Dual RRT");
  windowResolution = 2*boundarySize_m/((float) windowWidth_pix);
  if(windowWidth_pix < windowHeight_pix) {
    windowResolution = 2*boundarySize_m/((float) windowHeight_pix);   // using highest pixel gives lowest resolution
  }
  std::cout << "Window Resolution = " << windowResolution << std::endl;

  qInit = std::make_unique<rrt::rapidRandomTree>("StartPoint", nullptr, false);
  qGoal = std::make_unique<rrt::rapidRandomTree>("GoalPoint", qInit.get(), true);
  auto objects = qInit->getObjects();
  if(!objects.empty()) {
    for(const auto& it : objects) {
      placeObjectInMap(it, "obstacle");
    }
  }

  auto walls = qInit->getBorderWalls();
  if(!walls.empty()) {
    for(const auto& it : walls) {
      placeObjectInMap(it, "wall");
    }
  }

  placeRobotInMap(qInit->getRobotModel());
  auto goalPoint = qGoal->getTreeStart();
  placeGoalPointOnMap(goalPoint);

  std::cout << "Linear distance between start and end = " << rrt::rapidRandomTree::distance(qInit->getTreeStart(),
                                                                                            qGoal->getTreeStart()) << std::endl;

  counter = 0;
  startTime = rrt::system::timerEvent::getRunTime_ms();
}

void mainspace::shutdown() {
  watchWindow();

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

  if(configReader != nullptr) {
    rrt::system::jsonParser::deleteInstance();
    configReader = nullptr;
  }
  std::cout << "Shutting down the application." << std::endl;
}

void mainspace::signalHandler(int signum) {
  std::cout << "\nInterrupt signal (" << signum << ") received. Setting running to false" << std::endl;
  running = false;
  std::cout << "\nClose Visualizer Window to finish closing down the application" << std::endl;
}

void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2, const sf::Color color) {
  sf::VertexArray line(sf::LineStrip, 2);
  pt1 = convertPointToWindow(pt1);
  pt2 = convertPointToWindow(pt2);

  // define the vertices of the line
  line[0].position = sf::Vector2f(pt1.x(), pt1.y());
  line[1].position = sf::Vector2f(pt2.x(), pt2.y());

  // define the color of the line's points
  line[0].color = color;
  line[1].color = color;

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

//    window->clear(sf::Color::Black);

    // draw everything here...
    window->draw(line);

    // end the current frame
    window->display();
  }
}

void mainspace::placeObjectInMap(const rrt::objectNode& objectInMap, const std::string& objectName) {
  sf::RectangleShape object(sf::Vector2f(0, 0));
  object.setSize(sf::Vector2f(objectInMap.width/windowResolution,
                              objectInMap.height/windowResolution));

  object.setOrigin(objectInMap.width/(2*windowResolution), objectInMap.height/(2*windowResolution));

  auto convertedPoint = convertPointToWindow(objectInMap.location_m);
  object.setPosition(sf::Vector2f(convertedPoint.x(), convertedPoint.y()));
  object.setRotation(objectInMap.orientation*180.0f/((float) M_PI));

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

    // draw everything here...
    std::cout << "Drawing " << objectName << " " << objectInMap.objectId << std::endl;
    window->draw(object);

    // end the current frame
    window->display();
  }
}

void mainspace::placeRobotInMap(const rrt::objectNode& robotInMap) {
  sf::RectangleShape object(sf::Vector2f(0, 0));
  object.setSize(sf::Vector2f(robotInMap.width/windowResolution,
                              robotInMap.height/windowResolution));

  object.setOrigin(robotInMap.width/(2*windowResolution), robotInMap.height/(2*windowResolution));
  object.setFillColor(sf::Color(0, 0, 255));
  object.setOutlineColor(sf::Color(0, 0, 255));

  auto convertedPoint = convertPointToWindow(robotInMap.location_m);
  object.setPosition(sf::Vector2f(convertedPoint.x(), convertedPoint.y()));
  object.setRotation(robotInMap.orientation*180.0f/((float) M_PI));

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

    // draw everything here...
    std::cout << "Drawing Robot" << std::endl;
    window->draw(object);

    // end the current frame
    window->display();
  }
}

void mainspace::placeGoalPointOnMap(rrt::vector2f1& goalPt) {
  sf::CircleShape circle(0.050f/windowResolution);
  circle.setFillColor(sf::Color(255, 0, 0));
  circle.setOutlineColor(sf::Color(255, 0, 0));
  circle.setOrigin(sf::Vector2f(0.050f/windowResolution, 0.050f/windowResolution));

  auto convertedPoint = convertPointToWindow(goalPt);
  circle.setPosition(sf::Vector2f(convertedPoint.x(), convertedPoint.y()));

  if(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }

    // draw everything here...
    std::cout << "Drawing Goal Point" << std::endl;
    window->draw(circle);

    // end the current frame
    window->display();
  }
}

rrt::vector2f1 mainspace::convertPointToWindow(rrt::vector2f1 point) {
  rrt::vector2f1 convertedPoint;
  convertedPoint.x() = point.x()/windowResolution;
  convertedPoint.x() += ((float) windowWidth_pix/2.f);
  convertedPoint.y() = point.y()/windowResolution;
  convertedPoint.y() = ((float) windowHeight_pix/2.f) - convertedPoint.y();
  return convertedPoint;
}

void mainspace::mergeTrees() {
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
  auto connectingGoalNode = controlledGrower->getConnectingNeighbor();

  for(auto& it : goalTree) {          // Iterate through each node in the goal tree
    it.id += startTree.size();
    for(auto& it2 : it.neighbors) {   // Iterate through every neighbor in each node of goal tree
      it2 += startTree.size();
    }
  }

  if(controlledGrower->isGoalTree()) {
    connectingGoalNode.id += startTree.size();
    std::cout << "Connecting node id = " << connectingGoalNode.id << std::endl;

    // Last node of start tree should point towards the connecting goal node
    startTree.back().neighbors.emplace_back(connectingGoalNode.id);
  } else {
    startTree.at(connectingGoalNode.id).neighbors.emplace_back(goalTree.back().id);
  }

  // Create the map of possible paths for the robot in the environment
  pathMap = startTree;
  pathMap.insert(pathMap.end(), goalTree.begin(), goalTree.end());
}

void mainspace::incrementCounter() {
  counter++;
  if(counter >= maxNodes) {
    running = false;
    std::cout << "No viable path found in " << maxNodes << " to grow trees." << std::endl;
    double endTime = rrt::system::timerEvent::getRunTime_ms();
    std::cout << "Time ran = " << endTime - startTime << "ms" << std::endl;
  }
}

void mainspace::aStar() {
  std::cout << "\n" << std::endl;
  std::vector<pathNode*> openList;
  std::vector<pathNode*> closedList;
  rrt::vector2f1 goalLocation;
  goalLocation = qGoal->getTreeStart();

  // Add nodes in path to all Nodes and add heuristic
  allNodes.reserve(pathMap.size());
  for(const auto& it : pathMap) {
    auto newPathNode = pathNode();
    newPathNode.treePoint = it;
    newPathNode.id = it.id;
    newPathNode.h = rrt::rapidRandomTree::distance(goalLocation, it.location_m);
    allNodes.emplace_back(newPathNode);
  }

  // Add starting node to open list
  allNodes.front().inOpenList = true;
  allNodes.front().openListID = 0;
  openList.emplace_back(&allNodes.front());

  pathNode *q;           // q points to the current node
  uint64_t openListCounter;
  while(!openList.empty()) {
    // check cost of all nodes in open list and get lowest cost
    q = openList.front();
    for(auto& it : openList) {
      it->f = it->g + it->h;
      if(it->f < q->f) {
        q = it;           // q points to pathNode with lower cost
      }
    } // end for each iterator in openList

    // Check if reached goal node
    if(q->treePoint.goalNode) {
      // Add in stuff here, q holds goal node
      std::cout << "Path found" << std::endl;
      std::cout << "Node ID of goal = " << q->treePoint.id << std::endl;
      std::cout << "Node location of goal = " << std::endl << q->treePoint.location_m << std::endl;
      std::cout << "Parent of goal = " << q->parent << std::endl;
      createPath(*q);
      return;
    }

    // remove current from open list
    openList.erase(openList.begin() + (int) q->openListID);
    q->inOpenList = false;
    q->openListID = 0;

    // fix ids in openList
    if(!openList.empty()) {
      openListCounter = 0;
      for(auto it : openList) {
        it->openListID = openListCounter;
        openListCounter++;
      }
    }

    // add current to closed list
    q->inClosedList = true;
    q->closedListID = closedList.size();
    closedList.emplace_back(q);

    initNeighborCosts(*q);
    // Look at all neighbors in current node
    for(auto neighbor : q->treePoint.neighbors) {
      // Check if neighbor is in closed list
      if(!allNodes.at(neighbor).inClosedList) {
        allNodes.at(neighbor).f = allNodes.at(neighbor).g + allNodes.at(neighbor).h;
        // Check if neighbor in open list
        if(!allNodes.at(neighbor).inOpenList) {                 // Not in open list
          allNodes.at(neighbor).inOpenList = true;
          allNodes.at(neighbor).openListID = openList.size();
          openList.emplace_back(&allNodes.at(neighbor));        // Place neighbor in open list
        } else {                                                // In open list
          if(allNodes.at(neighbor).g < openList.at(allNodes.at(neighbor).openListID)->g) {
            openList.at(allNodes.at(neighbor).openListID)->g = allNodes.at(neighbor).g;  // Set new cost
            openList.at(allNodes.at(neighbor).openListID)->parent = q->id;               // Set new parent
          }
        }
      } // end not in closed list
    } // end loop through all neighbors of q (current node)
  } // end while open list not empty

  std::cout << "No path could be found" << std::endl;
  running = false;
}

void mainspace::initNeighborCosts(pathNode& parentNode) {
  for(const auto neighbor : parentNode.treePoint.neighbors) {
    // find cost from current node to node in open list
    allNodes.at(neighbor).g = parentNode.g + rrt::rapidRandomTree::distance(parentNode.treePoint.location_m,
                                                                            allNodes.at(neighbor).treePoint.location_m);
    allNodes.at(neighbor).parent = parentNode.id;
  }
}

void mainspace::createPath(pathNode& goalNode) {
  pathNode nextNode = goalNode;
  pathToGoal_m.clear();

  pathCreated = false;
  counter = 0;
  while(!pathCreated) {
    pathToGoal_m.emplace(pathToGoal_m.begin(), allNodes.at(nextNode.id).treePoint.location_m);
    if(nextNode.id == 0) {
      pathCreated = true;
    } else {
      nextNode = allNodes.at(nextNode.parent);
      counter++;
    }

    if(counter > maxNodes) {
      std::cout << "Could not create a path from RRT" << std::endl;
      running = false;
      return;
    }
  }
  std::cout << "Path has been Constructed" << std::endl;
}

void mainspace::smoothPath() {
  std::vector<rrt::vector2f1> smoothedPath_m, editablePath_m;
  rrt::vector2f1 locationToConsider_m;
  editablePath_m = pathToGoal_m;
  bool collision;

  // Init with start node
  smoothedPath_m.emplace_back(pathToGoal_m.front());

  bool pathSmoothed = false;
  while (!pathSmoothed) {
    // Implement greedy search method
    // On successful connection of node, add node to path
    locationToConsider_m = editablePath_m.front();
    for (uint64_t it = editablePath_m.size() - 1; it > 0; it--) {
      auto greedyPoint = editablePath_m.at(it);
      auto distanceBetweenNodes = rrt::rapidRandomTree::distance(locationToConsider_m, greedyPoint);
      // Check if collision to point
      int numSteps = (int) (distanceBetweenNodes / robotHeight_m) + 1;
      collision = false;
      if (numSteps > 1) {
        for (int i = numSteps; i > 1 ; i--) {
          float distance = distanceBetweenNodes * ((float) i);
          distance /= (float) numSteps;
          auto subPoint = rrt::rapidRandomTree::projectToPointOnLine(locationToConsider_m, greedyPoint, distance);
          if (rrt::rapidRandomTree::collisionDetection(subPoint, qInit->getRobotAndTransform(),
                                                       qInit->getObjectAndTransform(),
                                                       qInit->getWallsAndTransform())) {
            collision = true;
          }
        }
      } else {
        if (rrt::rapidRandomTree::collisionDetection(greedyPoint, qInit->getRobotAndTransform(),
                                                      qInit->getObjectAndTransform(),
                                                      qInit->getWallsAndTransform())) {
          collision = true;
        }
      }

      if (!collision) {
        smoothedPath_m.emplace_back(greedyPoint);
        if(greedyPoint == pathToGoal_m.back()) {
          pathSmoothed = true;
          break;
        }
        editablePath_m.erase(editablePath_m.begin(), editablePath_m.begin() + it - 1);
        break;
      }
    } // end looping path
  } // end while path has not been smoothed

  pathToGoal_m.clear();
  pathToGoal_m = smoothedPath_m;
} // end smooth path

float mainspace::getBearing_rad(rrt::vector2f1& currentPos, rrt::vector2f1& setPoint) {
  float dx = setPoint.x() - currentPos.x();
  float dy = setPoint.y() - currentPos.y();
  auto magnitude = (float) sqrt(pow(dx,2) + pow(dy,2));

  dx/= magnitude;
  dy/= magnitude;

  return atan2(dy,dx);
}

mainspace::pathNode::pathNode() {
  f = 0;
  g = 0;
  h = 0;
  id = 0;
  parent = 0;
  closedListID = 0;
  openListID = 0;
  inClosedList = false;
  inOpenList = false;
  treePoint = rrt::node();
}

void watchWindow() {
  while(window->isOpen()) {
    // check all the window's events that were triggered since the last iteration of the loop
    sf::Event event{};
    while (window->pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window->close();
    }
  }
}
