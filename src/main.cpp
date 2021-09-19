#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


// Helper funcitons
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
    angle = min(2*M_PI- angle, angle);

    if(angle > M_PI/4)
    {
      closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x and y onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int getLane(double d) {
  // Get lane number, assuming d does not change
  int lane_num;
  if (d > 0 && d < 4) {
    lane_num = 0;
  } else if (d > 4 && d < 8) {
    lane_num = 1;
  } else if (d > 8 && d < 12) {
    lane_num = 2;
  }
  return lane_num;
}


// Finite state machine functions
int chooseNextLane(int lane_num, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data);
vector<int> getLaneOptions(int lane_num);
int getFastestLane (vector<int> lane_options, double ego_vehicle_s, vector<vector<double>> sensor_data);
double getLaneSpeed (int lane_num, double ego_vehicle_s, vector<vector<double>> sensor_data);
vector<double> getClosestVehicle(int lane_num, double ego_vehicle_s, vector<vector<double>> sensor_data);

// Cost functionS
double cost(int original_lane, int intended_lane, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data);
//double goalDistanceCost(int original_lane, int intended_lane);
double inefficiencyCost(double goal_speed, double lane_speed);
double collisionCost(int intended_lane, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data);

// Select the best next lane based on cost function results
int chooseNextLane(int lane_num, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data) {
  vector<int> lane_options = getLaneOptions(lane_num);
  
  //std::cout << "ego_vehciel_speed: " + to_string(ego_vehicle_speed) << endl;
  
  // Initialize the minimum cost to be a large number
  double min_cost = 10000.0;
  double cost_thresh = 0.8; 
  double cost_i, current_lane_cost;
  
  /*
  Intended_lane is the fastest recheable lane (either current lane or adjacent lane), best_lane is the lane to choose,
  fastest_lane is the lane with the fastest traffic
  */
  int intended_lane, next_lane, fastest_lane;
  intended_lane = getFastestLane(lane_options, ego_vehicle_s, sensor_data); // Consider only the immediately reachable lanes
  next_lane = lane_num; // Keep lane is the default next state
  fastest_lane = getFastestLane({0, 1, 2}, ego_vehicle_s, sensor_data); // Consider all lanes
  
  double current_lane_speed, fastest_lane_speed, middle_lane_speed;
  current_lane_speed = getLaneSpeed(lane_num, ego_vehicle_s, sensor_data);
  fastest_lane_speed = getLaneSpeed(fastest_lane, ego_vehicle_s, sensor_data);
  middle_lane_speed = getLaneSpeed(1, ego_vehicle_s, sensor_data);
  
  //std::cout << "fastest_lane_speed: " + to_string(fastest_lane_speed) << endl;
  
  // Choose the lane with the smallest cost as the next lane
  for (int i = 0; i < lane_options.size(); i++) {
    cost_i = cost(lane_num, lane_options[i], ego_vehicle_s, ego_vehicle_speed, sensor_data);
    /*
    if (lane_options[i] = lane_num) {
      current_lane_cost = cost_i;
    }*/
    if (cost_i < min_cost) {
      min_cost = cost_i;
      next_lane = lane_options[i];
    }
  }
  // If even the minimum cost exceeds the threshold, keep lane to avoid collisions
  if (min_cost > cost_thresh) {
    next_lane = lane_num;
  }
  
  vector<double> distances = getClosestVehicle(fastest_lane, ego_vehicle_s, sensor_data);
  double dist_s_front = distances[0];
  double dist_s_rear = distances[1];
  /*
  Special case: when the adjacent lane is not faster than the current lane, but the faraway lane is way faster, 
  then choose the adjacent lane as the next lane to move to, so that the car can move to the fast lane
  */
  if ((fastest_lane == lane_num - 2) || (fastest_lane == lane_num + 2)) {
    /* Only consider lane change if the fastest lane is significantly faster or has far less traffic than the 
    current lane, and the middle lane is not too much slower */
    if ((fastest_lane_speed > current_lane_speed + 10)  || (dist_s_front > 40 && dist_s_rear > 15)) {
      if (middle_lane_speed > current_lane_speed - 5) {
        // Ignore inefficiency cost, consider collision cost only
        if (collisionCost(1, ego_vehicle_s, ego_vehicle_speed, sensor_data) < 0.4) {
          next_lane = 1; // Shift to the middle lane if unlikely to collide
        }
      }
    }
  }
  return next_lane;
}

vector<int> getLaneOptions(int lane_num) {
  vector<int> lane_options;
  if (lane_num == 1) {
    lane_options = {0, 1, 2}; // Car can turn left or right if it's in the middle lane
  } else if (lane_num == 0) {
    lane_options = {0, 1}; // Car can only turn right 
  } else if (lane_num == 2) {
    lane_options = {1, 2};
  }
  return lane_options;
}
// Find the fastest reachable lane
int getFastestLane (vector<int> lane_options, double ego_vehicle_s, vector<vector<double>> sensor_data) {
  // intended_lane is the fastest adjacent lane
  int intended_lane;
  
  // Initialize the maximum velocity to be a small number
  //double min_vel = 10000.0;
  double max_vel = -1.0;
  double vel_i;
  for (int i = 0; i < lane_options.size(); i++) {
    // Get the velocity of car in each lane
    vel_i = getLaneSpeed(lane_options[i], ego_vehicle_s, sensor_data);
    if (vel_i > max_vel) {
      max_vel = vel_i;
      intended_lane = lane_options[i];
    }
  }
  return intended_lane;
}

// Get the average speed of vehicles in a particular lane in mph
double getLaneSpeed (int lane_num, double ego_vehicle_s, vector<vector<double>> sensor_data) {
  int vehicle_counter = 0; // Count number of cars sensed in the lane
  double speed_sum = 0.0; // Summing speed values of sensed vehicles in the lane
  double lane_speed = 49.5; // If no vehicle sensed, then use target speed as the lane speed
  bool car_sensed = false;
  double dist_s; // Distance between the observed car and the ego vehicle in the s direction
  int car_lane; // Lane where the observed car is in
  
  for (int i = 0; i < sensor_data.size(); i++) {
    car_lane = getLane(sensor_data[i][6]);
    dist_s = sensor_data[i][5] - ego_vehicle_s;
    /* Any cars more than 10 meters behind or more than 50 meters in front of the ego vehicle are 
    considered to be irrelevant in calculating lane speed */
    if (car_lane == lane_num && dist_s > -10.0 && dist_s < 50.0) {
      speed_sum += sqrt(pow(sensor_data[i][3], 2.0) + pow(sensor_data[i][4], 2.0));
      vehicle_counter ++;
      car_sensed = true;
    }
  }
  if (car_sensed) {
    lane_speed = speed_sum/vehicle_counter*2.24; // Returns average speed of cars in mph
  }
  return lane_speed;
}

// Get the distance in s between the ego vehicle and the car closest to the ego vehicle in that lane, both in front and behind
vector<double> getClosestVehicle (int lane_num, double ego_vehicle_s, vector<vector<double>> sensor_data) {
  double min_dist1 = 10000.0;
  double min_dist2 = 10000.0;
  int car_lane;
  double dist_s_front, car_s, dist_s_rear;
  
  // If no cars detected in the lane, assume vehicles in the lane are very far from ego vehicle
  dist_s_front = 10001.0;
  dist_s_rear = 10001.0;
  
  for (int i = 0; i < sensor_data.size(); i++) {
    car_lane = getLane(sensor_data[i][6]);
    car_s = sensor_data[i][5];
    if (car_lane == lane_num) {
      if (car_s > ego_vehicle_s) {
        dist_s_front = car_s - ego_vehicle_s;
        if (dist_s_front < min_dist1) {
          min_dist1 = dist_s_front;
        }
      } else {
        dist_s_rear = ego_vehicle_s - car_s;
        if (dist_s_rear < min_dist2) {
          min_dist2 = dist_s_rear;
        }
      }
    }
  }
  
  return {dist_s_front, dist_s_rear};
}

double cost(int original_lane, int intended_lane, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data) {
  // TODO: implement collision cost and inefficiency cost
  // Can imitate provided code for collision cost
  double total_cost = 0.0;
  double lane_speed = getLaneSpeed(intended_lane, ego_vehicle_s, sensor_data);
  
  total_cost += inefficiencyCost(49.5, lane_speed);
  total_cost += collisionCost(intended_lane, ego_vehicle_s, ego_vehicle_speed, sensor_data);
  return total_cost;
}

double inefficiencyCost(double goal_speed, double lane_speed) {
    /*
    Cost becomes higher for trajectories with intended lane's speed (lane_speed) slower than vehicle's target speed (goal_speed). 
    */
    double cost = (goal_speed - lane_speed)/goal_speed;

    return cost;
}

double collisionCost(int intended_lane, double ego_vehicle_s, double ego_vehicle_speed, vector<vector<double>> sensor_data) {
  /*
  Cost becomes higher if there's a car in the intended lane very close to the ego vehicle in the s direction
  */
  double cost = 0.0;
  int car_lane;
  double car_s, car_speed; // The observed car's s coordinate and speed
  
  for (int i = 0; i < sensor_data.size(); i++) {
    car_lane = getLane(sensor_data[i][6]); // determine which lane the observed vehicle is in
    if (car_lane == intended_lane) {
      car_speed = sqrt(pow(sensor_data[i][3], 2.0) + pow(sensor_data[i][4], 2.0));
      // Assuming it takes 1 sec to complete a lane change, then this would be the car's position after 1 sec
      car_s = sensor_data[i][5] + car_speed; 
      // The closer the car is to the ego vehicle, the higher the cost
      cost += 40 * pow((car_s - ego_vehicle_s), -2.0);
      // cost += 5 * exp(-0.3 * (car_s - ego_vehicle_s));
      //cost += 3 / abs(car_s - ego_vehicle_s); 
    }
  }
  return cost;
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // Start in lane 1, which is the center lane (0 is left and 2 is right)
  int lane = 1;
  // Start at zero velocity and gradually accelerate
  double ref_vel = 0.0; // mph

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
          
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
          
          	int prev_size = previous_path_x.size();
          
          	if (prev_size > 0) {
              car_s = end_path_s;
            }
          
          	bool too_close = false; // True if too close to a car in front
          	//bool rear_too_close = false; // True if too close to a car at the back
          
          	// Find ref_v to use
          	for (int i = 0; i < sensor_fusion.size(); i++) {
              // Check if the car is in the same lane as the ego vehicle
              float d = sensor_fusion[i][6];
              if (d < (2+4*lane+2) && d > (2+4*lane-2)){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];
                
                // Calculate the check_car's future location
                check_car_s += (double)prev_size * 0.02 * check_speed;
                // If the check_car is within 30 meters in front, reduce ref_vel so that we don't hit it
                if (check_car_s > car_s && (check_car_s - car_s) < 30){
                  //ref_vel = 29.5;
                  too_close = true;
                  //int current_lane = lane;
                  lane = chooseNextLane(lane, car_s, car_speed, sensor_fusion);
                  //lane = 0;
                } 
              }
            }

          	std::cout << "too_close is: " + to_string(too_close) << endl;
          	json msgJson;
          
          	// Create a list of evenly spaced waypoints 30m apart
          	// Interpolate those waypoints later with spline and fill it in with more points
          	vector<double> ptsx;
          	vector<double> ptsy;
          
          	// Reference x, y, yaw states, either will be the starting point or end point of the previous path
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
          
          	// if previous size is almost empty, use the car as starting reference
          	if (prev_size < 2) {
              // Use two points that make the path tangent to the car
              double prev_car_x = car_x - 0.5 * cos(car_yaw);
              double prev_car_y = car_y - 0.5 * sin(car_yaw);
              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);
              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
          	// Use the previous path's end point as starting reference
          	else {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y - ref_y_prev , ref_x - ref_x_prev);
              // Use the two points that make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }
          
          	// Add evenly 30m spaced points ahead of the starting reference
          	vector<double> next_wp0 = getXY(car_s+40, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+70, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+100, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);
          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);
          
          	for (int i = 0; i < ptsx.size(); i++) {
              // shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
            }
          
          	// Create a spline
          	tk::spline s;
          	// Set (x,y) points to the spline
          	s.set_points(ptsx, ptsy);
          	// Define the actual (x,y) points we will use for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	// Start with all of the previous path points from last time
          	for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          	// Calculate how to break up spline points so that we travel at desired velocity
          	double target_x = 30.0; // 30.0 m is the distance horizon
          	double target_y = s(target_x);
          	double target_dist = sqrt(target_x*target_x + target_y*target_y);
          	double x_add_on = 0.0; // Related to the transformation (starting at zero)
          	// Fill up the rest of path planner after filling it with previous points, will always output 50
          	for (int i = 1; i <= 50-previous_path_x.size(); i++) {
              // Reduce speed if too close, add if no longer close
              if (too_close) {
                ref_vel -= .224;
              } else if (ref_vel < 49.5) {
                ref_vel += .112;
              }
              
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);
              
              x_add_on = x_point;
              
              double x_ref = x_point;
              double y_ref = y_point;
              
              // Rotate x, y back to normal
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
              
              x_point += ref_x;
              y_point += ref_y;
              
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
            
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
    std::cout << "lane is: " + to_string(lane) << std::endl;
    //std::cout << too_close << std::endl;
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}