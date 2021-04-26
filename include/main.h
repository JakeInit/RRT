/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_INCLUDE_MAIN_H
#define RRT_INCLUDE_MAIN_H

namespace mainspace {

struct pathNode {
  float f;    // Node Cost is g + h
  float g;    // Cost to get to current node from starting node
  float h;    // heuristic cost of node to goal. Will use euclidean distance
  uint64_t parent;
  uint64_t openListID;
  uint64_t closedListID;
  uint64_t id;
  bool inClosedList;
  bool inOpenList;
  rrt::node treePoint;
  pathNode();
};

void initialize();
void shutdown();
void signalHandler(int signum);
void incrementCounter();

void updateWindow(rrt::vector2f1 pt1, rrt::vector2f1 pt2);
void placeObjectInMap(const rrt::objectNode& objectInMap);
void placeRobotInMap(const rrt::objectNode& robotInMap);
void placeGoalPointOnMap(rrt::vector2f1& goalPt);
void mergeTrees();
void findPath();
void initNeighborCosts(pathNode& parentNode);
rrt::vector2f1 convertPointToWindow(rrt::vector2f1 point);

static bool running = false;
static rrt::system::jsonParser* configReader = nullptr;
static uint64_t windowHeight_pix;
static uint64_t windowWidth_pix;
static uint64_t maxNodes;
static float windowResolution;
static float robotHeight_m;
static float robotWidth_m;
static float boundarySize_m;
static float loopTime_ms;
static double startTime;
static std::vector<rrt::node> pathMap;
static std::vector<pathNode> allNodes;

uint16_t counter;

} // end namespace mainspace

#endif //RRT_INCLUDE_MAIN_H
