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

// local include
#include "rapidRandomTree.h"
#include "timerEvent.h"
#include "main.h"

#define ROBOTRADIUS 1.0
#define ATTEMPTS    500

using namespace mainspace;
std::unique_ptr<rrt::rapidRandomTree> qInit = nullptr;
std::unique_ptr<rrt::rapidRandomTree> qGoal = nullptr;
uint16_t counter = 0;

int main(int argc, char** argv) {
  running = true;
  signal(SIGINT, mainspace::signalHandler);

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
    std::cout << "test 1" << std::endl;
    rrt::vector2f1 newStartTreePoint = qInit->growTreeTowardsRandom();
    std::cout << "test 2" << std::endl;
    qGoal->growTreeTowardsPoint(newStartTreePoint);
    if(qGoal->goalReached()) {
      running = false;
      std::cout << "Trees are connected. Path from start to goal is possible" << std::endl;
      double endTime = rrt::system::timerEvent::getRunTime_ms();
      std::cout << "Time to find solution = " << endTime - startTime << "ms" << std::endl;
      std::cout << "Number of attempts to grow trees = " << counter << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    } else {
      counter++;
    }

    if(counter >= ATTEMPTS) {
      running = false;
      std::cout << "No viable path found in " << ATTEMPTS << " to grow trees." << std::endl;
      double endTime = rrt::system::timerEvent::getRunTime_ms();
      std::cout << "Time to find solution = " << endTime - startTime << "ms" << std::endl;
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
