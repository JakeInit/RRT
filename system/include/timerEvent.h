/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_SYSTEM_INCLUDE_TIMEREVENT_H
#define RRT_SYSTEM_INCLUDE_TIMEREVENT_H

#include <boost/asio/steady_timer.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio/io_service.hpp>

namespace rrt {
namespace system {

class timerEvent {
public:
  // Constructor
  timerEvent();
  // De-Constructor
  ~timerEvent();

  // public functions
  static void runBlockingTimer_ms(uint64_t duration);
  void runNonBlockingTimer_ms(uint64_t duration);
  void waitForTimerToFinish();
  bool stopTimer();
  static double getRunTime_ms();

  bool timerDone();

private:
  std::unique_ptr<boost::asio::steady_timer> timer_;
  std::unique_ptr<boost::asio::io_service> io;
  bool timer_ms_done;

};

} // end namespace system
} // end namespace rrt

#endif // RRT_SYSTEM_INCLUDE_TIMEREVENT_H
