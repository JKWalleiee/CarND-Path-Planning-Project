#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H
#include <iostream>
#include <vector>
#include <math.h>
#include "vehicle.h"

using namespace std;

class CostFunction {
public:
  const double COLLISION  = pow(10,7);
  const double DANGER     = pow(10,5);
  const double COMFORT    = pow(10,4);
  const double EFFICIENCY = pow(10,2);

  const double DESIRED_BUFFER = 1;

  vector<vector<double>> sensor_fusion;
  Vehicle *vehicle;

  /**
   * Constructor
   */
  CostFunction(Vehicle *v, vector<vector<double>> s);

  /*
   * Cost functions
   */
  double ChangeLane();
  double Inefficiency();
  double Collision();
  double Buffer();
  double Target();
  double Compute();
  
};

#endif
