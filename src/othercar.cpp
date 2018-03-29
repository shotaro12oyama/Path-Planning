#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "othercar.h"

Othercar::Othercar() {}
Othercar::Othercar(int id, int lane, double s, double d, double v) {
  this->id = id;  
  this->lane = lane;
  this->s = s;
  this->d = d;
  this->v = v;
}

Othercar::~Othercar() {}


vector<Othercar> detect_other_cars(double own_s, double own_d, vector<vector <double> > sensor_fusion) {
  // make vector which includes other_cars around own car
  vector<Othercar> other_cars;

  for(int i = 0; i < sensor_fusion.size(); i++ ) {
    double other_s = sensor_fusion[i][5];
    double other_rel_s = other_s - own_s;
    
    double other_d = sensor_fusion[i][6];
    double other_rel_d = other_d - own_d;
    
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double other_v = sqrt(vx*vx+vy*vy);
    
    int other_car_lane = static_cast<int>(other_d) / 4;
    // detect only close cars
    if (abs(other_rel_s) < 70) { 
      other_cars.push_back(Othercar(i, other_car_lane, other_rel_s, other_rel_d, other_v));
      cout << "i = " << i << " v = " << other_v <<  " lane = " << other_car_lane << " s = " << other_rel_s << endl; //<< " d = " << other_rel_d << " v = " << other_v << endl;
    }
  }
  
  return other_cars;
}


int select_next_lane(double own_d, double ref_vel, vector<Othercar> other_cars){
  vector<double> lane_select(3,0);
  // add cost to lane change
  int own_lane = static_cast<int>(own_d) / 4;
  for (int i=0; i<3; i++) {
    if (i == own_lane) {
        lane_select[i] += 0;
    } else {
      lane_select[i] += 4;
    }
  }

  for (auto itr = other_cars.begin(); itr != other_cars.end(); ++itr) {
    // add cost to the lane where other_car runs
    double cost_other_car = 2;
    lane_select[itr->lane] += cost_other_car;
    cout << "id = " << itr->id << " cost_other_car = " << cost_other_car << endl;

    // add cost for speed
    if(itr->s > 0) {
      double cost_speed = 4*(20-itr->v);
      lane_select[itr->lane] +=  cost_speed;
      cout << "id = " << itr->id << " cost_speed = " << cost_speed << endl;
    }
    if(abs(itr->s) <30 && itr->lane != own_lane) {
      double collision_avoid = 100;
      lane_select[itr->lane] +=  collision_avoid;
    }
  }

  int next_lane = min_element(lane_select.begin(), lane_select.end()) - lane_select.begin();
  // keep conservative decision making
  if(abs(next_lane - own_lane) > 1) {
      next_lane = own_lane;
  }


  for (int i=0; i<3; i++) {
    cout << "lane_select[" << i << "] = " << lane_select[i] << endl;
  }
  return next_lane; 
}


double adjust_ref_vel(int next_lane, double ref_vel, double own_speed, vector<Othercar> other_cars) {
  double vel = ref_vel;
  bool too_close = false;
  for (auto itr = other_cars.begin(); itr != other_cars.end(); ++itr) {
    if (itr->lane == next_lane) {
      if (itr->s > 0 && itr->s < 30 && itr->v < own_speed) {
        vel -= .2;
        too_close = true;
      }
    }
  }
  if (vel < 49.5 && too_close != true) {
    vel += .2;
  }
  
  return vel;

}

double logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}





