/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_INCLUDE_MAIN_H
#define RRT_INCLUDE_MAIN_H

namespace mainspace {

void shutdown();
void signalHandler(int signum);

static bool running = false;

} // end namespace mainspace

#endif //RRT_INCLUDE_MAIN_H
