#ifndef OTHERCAR_H
#define OTHERCAR_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Othercar {
public:
  int id;

  int lane;

  double s;

  double d;

  double v;

  /**
  * Constructor
  */  
  Othercar();
  Othercar(int id, int lane, double s, double d, double v);

  /**
  * Destructor
  */
  virtual ~Othercar();


};

vector<Othercar> detect_other_cars(double own_s, double own_d, vector<vector <double> > sensor_fusion);
int select_next_lane(double own_d, double ref_vel, vector<Othercar> other_cars);
double adjust_ref_vel(int next_lane, double ref_vel, double own_speed, vector<Othercar> other_cars);
double logistic(double x);


#endif