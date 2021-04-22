/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

// local includes
#include "timerEvent.h"

// /usr/include
#include <iostream>
#include <thread>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <ctime>

namespace rrt {
namespace system {

using namespace boost::asio;

// Constructor
timerEvent::timerEvent() : timer_(nullptr), io(nullptr) {
  io = std::make_unique<io_service>();
  timer_ = std::make_unique<boost::asio::steady_timer>(*io);
  timer_ms_done = true;
}

// De-Constructor
timerEvent::~timerEvent() {
  if(timer_) {
    timer_.reset();
  }

  if(io) {
    io.reset();
  }
}

void timerEvent::runBlockingTimer_ms(uint64_t duration) {
  io_service ioservice;
  steady_timer timer{ioservice, std::chrono::milliseconds{duration}};
  timer.wait();
}

void timerEvent::runNonBlockingTimer_ms(uint64_t duration) {
  timer_ms_done = false;
  timer_->expires_from_now(std::chrono::milliseconds{duration});
  timer_->async_wait([=](const boost::system::error_code &ec) {
    if (ec) {
      std::cout << "Timer Error: " << ec << std::endl;
      timer_->cancel();
    }
    timer_ms_done = true;
  });
}

void timerEvent::waitForTimerToFinish() {
  auto timeLeft = timer_->expires_from_now();
  if(timeLeft.count() > 0) {
    boost::this_thread::sleep_for(boost::chrono::nanoseconds(timeLeft.count()));
  }

  io->run();
  io->reset();
}

bool timerEvent::stopTimer() {
  io->stop();
  io->reset();
  timer_ms_done = true;
  return timer_ms_done;
}

bool timerEvent::timerDone() {
  if(!timer_ms_done) {
    if (io->poll()) {
      io->reset();
    }
  }

  return timer_ms_done;
}

double timerEvent::getRunTime_ms() {
  struct timespec time_struct{};
  clock_gettime(CLOCK_MONOTONIC, &time_struct);
  double time_ms = static_cast<double>(time_struct.tv_sec) * 1000.0;
  time_ms += static_cast<double>(time_struct.tv_nsec) * 0.000001;
  return time_ms;
}

} // end namespace system
} // end namespace rrt