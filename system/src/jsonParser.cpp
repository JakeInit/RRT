/*
  Author:   Jacob Morgan
  Date:     04/21/21
  Company:  UNCC
  Extra:
*/

#include "jsonParser.h"
#include <json/value.h>
#include <json/json.h>
#include <fstream>
#include <iostream>

namespace rrt{
namespace system {

jsonParser* jsonParser::instance = nullptr;

//  Constructor
jsonParser::jsonParser() {
  paramsLoaded = false;

  parseConfigValues();
}

void jsonParser::parseConfigValues() {
  if(hasLoadedConfig()) {
    return;
  }

  Json::Value parameters;
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  std::string jsonFileName = "config";
  JSONCPP_STRING error;

  std::string jsonPath = (std::string) PROJECTDIR + "/json/" + jsonFileName + ".json";
  std::ifstream robot_file(jsonPath, std::ifstream::binary);
  if (!parseFromStream(builder, robot_file, &parameters, &error)) {
    std::cout << "issue parsing " << jsonFileName << ".json" << std::endl;
    std::cout << "error: " << error << std::endl;
    exit(EXIT_SUCCESS);
  }
  if (parameters.empty()) {
    std::cout << "json file " << jsonFileName << ".json not found" << std::endl;
    exit(EXIT_SUCCESS);
  }

  parametersForRobot.dims.width_m = parameters["robot"]["width_m"].asFloat();
  parametersForRobot.dims.height_m = parameters["robot"]["height_m"].asFloat();
  parametersForRobot.buffer_m = parameters["robot"]["buffer_m"].asFloat();

  // User defined objects
  objects.clear();
  for(const auto& object : parameters["objects"]) {
    std::vector<std::pair<float, float>> vertices;
    std::pair<float, float> vertex;
    for(const auto& pair : object) {
      vertex.first = pair[0].asFloat();
      vertex.second = pair[1].asFloat();
      vertices.emplace_back(vertex);
    }
    objects.emplace_back(vertices);
  }

  parametersForSystem.pathSmootherOn = parameters["system"]["pathSmootherOn"].asBool();
  parametersForSystem.UserDefinedObjectsOn = parameters["system"]["UserDefinedObjectsOn"].asBool();
  parametersForSystem.loop_frequency_Hz = parameters["system"]["loop_frequency_Hz"].asFloat();
  parametersForSystem.stepDistance_m = parameters["system"]["stepDistance_m"].asFloat();
  parametersForSystem.boundaryHeight_m = parameters["system"]["boundaryHeight_m"].asFloat();
  parametersForSystem.boundaryWidth_m = parameters["system"]["boundaryWidth_m"].asFloat();
  parametersForSystem.maxObjectSize_m = parameters["system"]["maxObjectSize_m"].asFloat();
  parametersForSystem.maxNodes = parameters["system"]["maxNodes"].asUInt64();
  parametersForSystem.visualizerHeight_pix = parameters["system"]["visualizerHeight_pix"].asUInt64();
  parametersForSystem.visualizerWidth_pix = parameters["system"]["visualizerWidth_pix"].asUInt64();
  parametersForSystem.maxObjects = parameters["system"]["maxObjects"].asUInt64();
  parametersForSystem.minObjects = parameters["system"]["minObjects"].asUInt64();

  paramsLoaded = true;
}

bool jsonParser::hasLoadedConfig() const {
  return paramsLoaded;
}

bool jsonParser::isConfigValid() const {
  return paramsLoaded;
}

jsonParser *jsonParser::getInstance() {
  if (instance == nullptr) {
    std::cout << "Creating new instance of json parser" << std::endl;
    instance = new jsonParser();
  }

  return instance;
}

void jsonParser::deleteInstance() {
  if (instance != nullptr) {
    delete instance;
    instance = nullptr;
    std::cout << "deleted instance of jsonParser" << std::endl;
  }
}

jsonParser::robotConfiguration::robotConfiguration() {
  dims.height_m = 0;
  dims.width_m = 0;
  buffer_m = 0;
}

jsonParser::systemConfiguration::systemConfiguration() {
  pathSmootherOn = false;
  loop_frequency_Hz = 0;
  stepDistance_m = 0;
  boundaryHeight_m = 0;
  boundaryWidth_m = 0;
  maxNodes = 0;
  visualizerHeight_pix = 0;
  visualizerWidth_pix = 0;
  maxObjectSize_m = 0;
  maxObjects = 0;
  minObjects = 0;
}

} // end namespace system
} // end namespace rrt

