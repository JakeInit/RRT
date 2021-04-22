/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

// local includes
#include "asynchronous.h"

// /usr/local/include
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/atomic.hpp>
#include <exception>
#include <iostream>
#include <sys/prctl.h>
#include <thread>

namespace rrt {
namespace system {

asynchronous::asynchronous() : boostThread(nullptr), running(false) {}

asynchronous::~asynchronous() {
  setRunning(false);
  join();
}

bool asynchronous::isRunning() {
  boost::lock_guard<boost::mutex> lock{mutex};
  return running;
}

void asynchronous::setRunning(bool setTo) {
  boost::lock_guard<boost::mutex> lock{mutex};
  running = setTo;
}

void asynchronous::join() {
  if(boostThread) {
    boostThread->join();
    boostThread.reset();
  }
}

void asynchronous::join(unsigned int time_s) {
  if(boostThread) {
    struct timespec ts{};

    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      join();
    } else {

      boost::posix_time::time_duration td = boost::posix_time::seconds(time_s);
      try {
        boostThread->timed_join(td);
      } catch (std::exception &e) {
        std::cout << e.what() << std::endl;
      }
    }
  }

  boostThread.reset();
}

void asynchronous::runBase() {
  run();
}

void asynchronous::create(const std::string& threadName) {
  setRunning(true);
  std::string name;

  if (threadName.size() > 15) {
    std::cout << "Thread name can only be 15 characters long." << std::endl;
    name = threadName.substr(0, 14);
  } else {
    name = threadName;
  }

  std::cout << "Creating Threaded class with name " << name << std::endl;
  boostThread = std::make_unique<boost::thread>(&asynchronous::runBase, this);
  prctl(PR_SET_NAME, name.c_str(), 0,0,0);
}

} // end namespace system
} // end namespace rrt

