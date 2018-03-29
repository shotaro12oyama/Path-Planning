#ifndef BEHAVIOR_H
#define BEHAVIOR_H
#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "othercar.h"

using namespace std;

class Behavior {
public:
  int cost;

  float dt = 0.02; // every .02 seconds
  
  vector<float> s;
  
  vector<float> d;

  /**
  * Constructor
  */  
  Behavior();
  Behavior(float car_x, float car_y, float car_v);

  /**
  * Destructor
  */
  virtual ~Behavior();

  vector<string> state_in(vector<vector <double> > sensor_fusion);

};

#endif