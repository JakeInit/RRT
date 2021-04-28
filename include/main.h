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

void placeObjectInMap(const rrt::objectNode& objectInMap, const std::string& objectName);
void placeRobotInMap(const rrt::objectNode& robotInMap);
void placeGoalPointOnMap(rrt::vector2f1& goalPt);
void mergeTrees();
void aStar();
void createPath(pathNode& goalNode);
void initNeighborCosts(pathNode& parentNode);
void smoothPath();
float getBearing_rad(rrt::vector2f1& currentPos, rrt::vector2f1& setPoint);
rrt::vector2f1 convertPointToWindow(rrt::vector2f1 point);

static bool running = false;
static bool pathCreated = false;
static bool pathSmootherOn = false;
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
static std::vector<rrt::vector2f1> pathToGoal_m;

uint16_t counter;

} // end namespace mainspace

#endif //RRT_INCLUDE_MAIN_H
