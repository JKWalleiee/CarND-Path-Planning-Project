#include "cost_function.h"
#include <iostream>
#include <iostream>
#include <math.h>
#include "vehicle.h"

const double MS_TO_MPH = 2.23694;

const double MPH_TO_MS = 0.44704;

CostFunction::CostFunction(Vehicle *v, vector<vector<double>> s){
  vehicle = v;
  sensor_fusion = s;
}

double CostFunction::Compute(){
  
  //compute cost
  double cost = 0;
  cost += ChangeLane();  
  cost += Inefficiency();
  cost += Collision();
  cost += Buffer();
  cost += Target();
  
  return cost;
}

// Computes cost proportional to the difference between the ego-vehicle's speed
// and the vehicles's speed in the end lane
double CostFunction::Target(){
  double cost =0;
  if(!vehicle->collider.collision){
    return 0;
  }
  double diff = (vehicle->collider.target_speed - vehicle->speed)/vehicle->collider.target_speed;
  cost = pow(diff,2) * EFFICIENCY;      
  return cost;
}

//Computes a cost to penalizes the lane change.
double CostFunction::ChangeLane(){
  int end_lane = vehicle->trajectory.lane_end;
  int start_lane = vehicle->trajectory.lane_start;
  double cost = 0;
  if(start_lane != end_lane){
    cost = COMFORT;
  } 
  
  return cost;
}

// Computes a cost to penalize lower speeds compared to the maximum speed allowed ("less efficient").
double CostFunction::Inefficiency(){
  double cost = 0;
  double diff = (49.5 - vehicle->update.target_vel)/49.5;
  cost = pow(diff,2) * EFFICIENCY;  
  return cost;
}

// Computes a cost that penalizes the states in which the collision risk is more imminent
double CostFunction::Collision(){  
  double cost = 0;
  if(vehicle->collider.collision){
    
    //distance divided by the relative speed
    double time_to_collide = abs(vehicle->collider.distance)/(abs(vehicle->speed)*MPH_TO_MS);
    cost = exp(-pow(time_to_collide,2))*COLLISION;
    //changing lane    
    if(vehicle->trajectory.lane_end != vehicle->trajectory.lane_start){
      if(time_to_collide > DESIRED_BUFFER){
        //safe to change lane
        cost /= 10;
      } 
    }
  }
  return cost;
}

// Computes a cost according to how long the ego vehicle is for the other vehicles in the lane
double CostFunction::Buffer(){
  double cost = 0;

  if(vehicle->collider.closest_collis == 10000){
    return 0;
  }

  double time_steps = abs(vehicle->collider.closest_collis)/(abs(vehicle->speed)*MPH_TO_MS);

  
  if(time_steps > DESIRED_BUFFER){
    return 0;
  }

  double multiplier = 1.0 - pow((time_steps / DESIRED_BUFFER),2);
  cost = multiplier * DANGER;
  if(vehicle->collider.closest_collis < 0){
    //car behind
    cost /= 10;
  }       
  return cost;
}
