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

#define ROBOTRADIUS 0.50

using namespace mainspace;
std::unique_ptr<rrt::rapidRandomTree> qInit = nullptr;
std::unique_ptr<rrt::rapidRandomTree> qGoal = nullptr;

int main(int argc, char** argv) {
  running = true;
  signal(SIGINT, mainspace::signalHandler);

  rrt::system::timerEvent timer;
  const float loopTime_ms = 1000;

  qInit = std::make_unique<rrt::rapidRandomTree>("StartPoint", ROBOTRADIUS);
  qGoal = std::make_unique<rrt::rapidRandomTree>("GoalPoint", ROBOTRADIUS);

  std::cout << "Entering the Running Loop" << std::endl;
  while(running) {
    if(timer.timerDone()) {
      timer.runNonBlockingTimer_ms(std::floor(loopTime_ms));
    } else {
      timer.waitForTimerToFinish();
      continue;
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
