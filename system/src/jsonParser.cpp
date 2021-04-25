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

  paramtersForSystem.loop_frequency_Hz = parameters["system"]["loop_frequency_Hz"].asFloat();
  paramtersForSystem.stepDistance_m = parameters["system"]["stepDistance_m"].asFloat();
  paramtersForSystem.boundaryHeight_m = parameters["system"]["boundaryHeight_m"].asFloat();
  paramtersForSystem.boundaryWidth_m = parameters["system"]["boundaryWidth_m"].asFloat();
  paramtersForSystem.maxNodes = parameters["system"]["maxNodes"].asUInt64();
  paramtersForSystem.visualizerHeight_pix = parameters["system"]["visualizerHeight_pix"].asUInt64();
  paramtersForSystem.visualizerWidth_pix = parameters["system"]["visualizerWidth_pix"].asUInt64();

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
  } else {
    std::cout << "Could not delete jsonParser instance since already nullptr" << std::endl;
  }
}

jsonParser::robotConfiguration::robotConfiguration() {
  dims.height_m = 0;
  dims.width_m = 0;
}

jsonParser::systemConfiguration::systemConfiguration() {
  loop_frequency_Hz = 0;
  stepDistance_m = 0;
  boundaryHeight_m = 0;
  boundaryWidth_m = 0;
  maxNodes = 0;
  visualizerHeight_pix = 0;
  visualizerWidth_pix = 0;
}

} // end namespace system
} // end namespace rrt

