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

void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2);
void placeObjectInMap(const rrt::objectNode& objectInMap);
void placeRobotInMap(const rrt::objectNode& robotInMap);
void placeGoalPointOnMap(rrt::vector2f1& goalPt);
rrt::vector2f1 convertPointToWindow(rrt::vector2f1 point);

static bool running = false;
static rrt::system::jsonParser* configReader = nullptr;
static uint64_t windowHeight_pix;
static uint64_t windowWidth_pix;
static float windowResolution;
static float robotHeight_m;
static float robotWidth_m;
static float boundarySize_m;

uint16_t counter;

} // end namespace mainspace

#endif //RRT_INCLUDE_MAIN_H
