/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#ifndef RRT_SYSTEM_INCLUDE_JSONPARSER_H
#define RRT_SYSTEM_INCLUDE_JSONPARSER_H

#include <string>
#include <vector>

namespace rrt {
namespace system {

class jsonParser {
public:
  struct robotConfiguration {
    struct robotDimensions {
      float width_m;
      float height_m;
    } dims{};
    float buffer_m;
    robotConfiguration();
  };

  struct systemConfiguration {
    bool pathSmootherOn;
    bool UserDefinedObjectsOn;
    float loop_frequency_Hz;
    float stepDistance_m;
    float minStepDistance_m;
    float boundaryHeight_m;
    float boundaryWidth_m;
    float maxObjectSize_m;
    uint64_t maxNodes;
    uint64_t maxObjects;
    uint64_t minObjects;
    uint64_t visualizerHeight_pix;
    uint64_t visualizerWidth_pix;
    systemConfiguration();
  };

  bool hasLoadedConfig() const;
  bool isConfigValid() const;
  static jsonParser* getInstance();
  static void deleteInstance();

  robotConfiguration parametersForRobot;
  systemConfiguration parametersForSystem;

  // Each vector iterator is an object
  //  each iterator in the sub vector is a coordinate of the object
  //    coordinates are stored as x, y pairs
  std::vector<std::vector<std::pair<float, float>>> objects;

private:
  // Constructor
  jsonParser();

  // De-Constructor
  ~jsonParser() = default;

  void parseConfigValues();

  bool paramsLoaded;

  static jsonParser *instance;
};

} // end namespace system
} //  end namespace rrt

#endif // RRT_SYSTEM_INCLUDE_JSONPARSER_H
