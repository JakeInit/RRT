/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_SYSTEM_INCLUDE_ASYNCHRONOUS_H
#define RRT_SYSTEM_INCLUDE_ASYNCHRONOUS_H

#include <boost/thread/thread.hpp>

namespace rrt {
namespace system {

class asynchronous {
public:
  // Constructor
  asynchronous();

  // Deconstructor
  virtual ~asynchronous();

//  template<class M>
//  void create(asynchronous& object, const std::string& threadName) {
  void create(const std::string& threadName);
  bool isRunning();
  void setRunning(bool setTo);
  void join(unsigned int time_s);
  void join();

private:
  boost::mutex mutex;
  std::unique_ptr<boost::thread> boostThread;

  void runBase();

protected:
  bool running;
  virtual void run() = 0;
};
} // end namespace system
} // end namespace rrt

#endif // RRT_SYSTEM_INCLUDE_ASYNCHRONOUS_H
