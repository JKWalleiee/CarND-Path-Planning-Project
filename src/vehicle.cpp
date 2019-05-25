#include "vehicle.h"
#include <iostream>
#include <iostream>
#include <math.h>
#include "cost_function.h"

const double MS_TO_MPH = 2.23694;

const double MPH_TO_MS = 0.44704;

Vehicle::Vehicle(int lane, double target_speed){
    ref_speed = target_speed;
    ref_lane = lane;
}
  
void Vehicle::Update(double ax, double ay, double as, double ad, double ayaw, double aspeed, int lane, double target_speed, double delta){
  
  //update vehicle position and configuration data
  x = ax;
  y = ay;
  s = as;
  d = ad;
  yaw = ayaw;
  speed = aspeed;
  delta_time = delta;

  ref_speed = target_speed;
  ref_lane = lane;

  //reset and update configuration data
  reset_data();
}

void Vehicle::reset_data(){
  //clean data

  //reset trajectory
  trajectory.lane_start = ref_lane;
  trajectory.lane_end = ref_lane;
  trajectory.target_speed = ref_speed;

  //reset update
  update.ref_vel = ref_speed;
  update.lane = ref_lane;
  update.target_vel = 49.50;
  collider.collision = false;
  collider.distance = 10000;
  collider.closest_collis = 10000;
  collider.target_speed = 0;
}


void Vehicle::choose_next_state(vector<vector<double>> sensor){
  vector<States> states;

  //select next state from the Finite-state machine
  states.push_back(KL);
  if(state == PLCL)
  {
    states.push_back(LCL);
    states.push_back(PLCL);
  } 
  else if(state == PLCR)
  {
    states.push_back(LCR);
    states.push_back(PLCR);
  } 
  else 
  {    
    if(ref_lane != 0)
    {      
      //check if the lane change is over before go to the LCL state
      if( (d < (2+4*(ref_lane)+2) ) && (d > (2+4*(ref_lane)-2) )
          && (speed > 20) )
      { 
	      states.push_back(PLCL);    
      }      
    }
    if(ref_lane != 2)
    {
      //check if the lane change is over before go to the LCR state
      if( (d < (2+4*(ref_lane)+2) ) && (d > (2+4*(ref_lane)-2) )
          && (speed > 20) )
      {
	      states.push_back(PLCR);    
      }
    }
  }

  States next_state = KL;
  double  min_cost = 100000;

  //compute cost of all reachable states
  for(int i =0; i < states.size(); i++)
  {
    States n_state = states[i];
    //prepare state
    reset_data();
    realize_state(n_state, sensor);    
    CostFunction cost = CostFunction(this, sensor);
    double state_cost = cost.Compute();  
    if(state_cost < min_cost)
    {
      next_state = n_state;
      min_cost = state_cost;
    }
  }
  
  //update state
  state = next_state;
  reset_data();
  realize_state(state, sensor);
  
  //update speed
  if( (!collider.collision) && (ref_speed < update.target_vel) && (ref_speed < 49.5) )
  {
    update.ref_vel += 0.224;
  } 
  else if( (ref_speed > update.target_vel) && (ref_speed > 0) )
  {
    update.ref_vel -= 0.224;
  }
  
}

void Vehicle::realize_state(States n_state, vector<vector<double>> sensor_fusion){
  
  switch(n_state)
  {
    case KL: 
    {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane;
      update.lane = ref_lane;
      break;    
    }
    case PLCL:
    {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane - 1;
      update.lane = ref_lane;
      break;
    }
    case LCL:
    {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane - 1;
      update.lane = ref_lane - 1;
      break;
    }    
    case PLCR:
    {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane + 1;
      update.lane = ref_lane;
      break;
    }
    case LCR:
    {
      trajectory.lane_start = ref_lane;
      trajectory.lane_end = ref_lane + 1;
      update.lane = ref_lane + 1;
      break;
    } 
    default:
      std::cout << "ERROR - Invalid State\n";
  }

  //check lane
  if(trajectory.lane_end < 0)
  {
    trajectory.lane_end = 0;
  } 
  else if(trajectory.lane_end > 2)
  {
    trajectory.lane_end = 2;
  }

  if(trajectory.lane_start < 0)
  {
    trajectory.lane_start = 0;
  } 
  else if(trajectory.lane_start > 2)
  {
    trajectory.lane_start = 2;
  }

  if(update.lane < 0)
  {
    update.lane = 0;
  } 
  else if(update.lane > 2)
  {
    update.lane = 2;
  }

  //Safety variables
  double safety_speed_in_front = 0;
  double safety_distance_in_front = 10000;
  double safety_speed_lane_front = 0;
  double safety_distance_lane_front = 10000;
  double safety_speed_lane_back = 0;
  double safety_distance_lane_back = -10000;

  //compute collision on start and end lane
  for(int i=0; i < sensor_fusion.size(); i++)
  {
    //car is in in the same lane (start lane)
    float car_d = sensor_fusion[i][6];
    //Safety check for speed of car in the same lane
    if( (car_d < (2+4*(trajectory.lane_start)+2) ) 
        && (car_d > (2+4*(trajectory.lane_start)-2) ) )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += ((double) delta_time*check_speed);
      
      //check s values greater than the ego vehicle and s gap
      double dist_to_collision = (check_car_s - s); 

      if( (check_car_s >= s) && (dist_to_collision < 30) )
      {
        if(safety_distance_in_front > dist_to_collision)
        {
          safety_speed_in_front = check_speed*MS_TO_MPH-2;
          safety_distance_in_front = dist_to_collision;
        }	
      }
    }

    //check for car in the goal lane (end lane)
    if( (car_d < (2+4*(trajectory.lane_end)+2) )
        && (car_d>(2+4*(trajectory.lane_end)-2) ) )
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      
      check_car_s += ((double) delta_time*check_speed);
      
      //check s values greater than the ego vehicle and s gap
      double dist_to_collision = (check_car_s - s);

      if( ( trajectory.lane_end!=trajectory.lane_start && abs(dist_to_collision) < 30 )
	        || ( check_car_s >= s && dist_to_collision < 30 ) )
      {	
        if( collider.distance > abs(dist_to_collision) )
        {	  
          collider.distance = abs(dist_to_collision);
          collider.collision = true;
          collider.closest_collis = abs(dist_to_collision);
          collider.target_speed = check_speed*MS_TO_MPH;	 

          if(abs(dist_to_collision) > 30)
          {
            //change target speed
            if(check_car_s >= s)
            {
              //car in front
              update.target_vel = check_speed*MS_TO_MPH-2;
              if( safety_distance_lane_front > dist_to_collision )
              {
                safety_speed_lane_front = check_speed*MS_TO_MPH;
                safety_distance_lane_front = dist_to_collision;
              }
            } 
            else 
            {
              //car is behind
              update.target_vel = check_speed*MS_TO_MPH+2;
              if(safety_distance_lane_back < dist_to_collision)
              {
                safety_speed_lane_back = check_speed*MS_TO_MPH;
                safety_distance_lane_back = dist_to_collision;
              }
            }
          }
        }
      } 
      else if(!collider.collision
		            && collider.closest_collis > dist_to_collision)
      {
        collider.closest_collis = dist_to_collision;
        collider.target_speed = check_speed*MS_TO_MPH;
      }
    }   
  }

  //Safety Speed check
  if(n_state==PLCL || n_state==PLCR){
    //safety speed update
    if( (safety_speed_lane_back!= 0) && (update.target_vel < safety_speed_lane_back) )
    {
      update.target_vel = safety_speed_lane_back;
    }
    if( (safety_speed_lane_front!=0) && (update.target_vel > safety_speed_lane_front) )
    {
      update.target_vel = safety_speed_lane_front;
    }
  } 

  if( (safety_speed_in_front!=0) && (update.target_vel > safety_speed_in_front) )
  {
    update.target_vel = safety_speed_in_front-2;
  }
   
}
