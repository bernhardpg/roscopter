//
// Created by lars on 4/23/20.
//

#include "configParser.h"
#include "constants.h"
#include <map>
#include <fstream>
#include <stdexcept>
#include <pystring/pystring.h>

#include <stdlib.h>

using namespace std;

pair<string, string> splitInTwo(string val) {
  string arg;
  string::size_type pos = val.find('=');
  if(val.npos != pos) {
    arg = val.substr(pos + 1);
    val = val.substr(0, pos);
  }
  return make_pair(val, arg);
}

string getValueForKey(string key, map<string, string> configMap){  // TODO: Write this generic?
  auto pos = configMap.find(key);
  if (pos == configMap.end()) {
    throw std::invalid_argument( "Could not find parameter: " + key);
  } else {
    return pos->second;
  }
}

static std::string matrixToString(const Eigen::MatrixXd& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

std::vector<double> getInitialStateVector(map<string, string> configMap){
  std::vector<std::string> vec;
  pystring::split(getValueForKey("initialState", configMap), vec, ",");
  std::vector<double> doubleVector(vec.size());
  std::transform(vec.begin(), vec.end(), doubleVector.begin(), [](const std::string& val)
  {
    return std::stod(val);
  });
  return doubleVector;
}

Config ConfigParser::getConfig(){
  Config conf{};
  string line ;
  ifstream infile;
  infile.open (NoroboConstants::configName);
  map<string, string> configMap;

  while(getline(infile,line)) // To get you all the lines.
  {
    pair<string, string> p = splitInTwo(line);
    configMap.insert({get<0>(p), get<1>(p)});
    std::cout << "key:" <<get<0>(p) << " = " << get<1>(p);
  }
  infile.close();
  conf.windowSize = std::stoi(getValueForKey("windowSize", configMap));
  conf.stateDebug = std::stoi(getValueForKey("stateDebug", configMap));
  conf.planningTimesteps = std::stoi(getValueForKey("planningTimesteps", configMap));
  conf.planningDt = std::stod(getValueForKey("planningDt", configMap));
  conf.enableRollingWindow =std::stoi(getValueForKey("enableRollingWindow", configMap));
  conf.initialState = Eigen::Map<Eigen::Matrix<double, 12, 1> >(getInitialStateVector(configMap).data());

  std::cout << "initialState = " << matrixToString(conf.initialState);
  return conf;
}
