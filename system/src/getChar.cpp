/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

// /usr/include
#include <iostream>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>
#include <poll.h>

// local includes
#include "getChar.h"

namespace rrt {
namespace system {

unsigned char getChar(double timeout_ms) {
  unsigned char input = '\0';

  fd_set readSet;
  FD_ZERO(&readSet);
  FD_SET(STDIN_FILENO, &readSet);

  unsigned int timeout_us = (unsigned long) (timeout_ms*1000);
  struct timeval tv = {0, timeout_us};

  if (select(STDIN_FILENO+1, &readSet, NULL, NULL, &tv) < 0) {
    perror("select");
  }

  bool fdIsSet = FD_ISSET(STDIN_FILENO, &readSet);

  if(fdIsSet) {
    std::cin >> input;
  }

  return input;

}

} // end namespace system
} // end namespace rrt
