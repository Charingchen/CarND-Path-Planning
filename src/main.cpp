#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class Vehicle{
public:
    // Constructors
//    Vehicle();
//    Vehicle(int lane, float s, float v, string state="CS");
    
    // Destructor
//    virtual ~Vehicle();
    
    int lane, s;
    
    float v;
    // F: front RB: right back RF: right front LB: left back LF: left front
    string state;
    
//    void cal_mag_v (double v_x, double v_y);
    
    void cal_mag_v(double v_x, double v_y, double prev_size){
        double target_mag_v = sqrt(v_x*v_x + v_y*v_y);
        // Prediction the future location
        s += (double)prev_size * 0.02 * target_mag_v;
    }
};

//class FSM {
//public:
//    int state;
//    bool slow_down;
//
//
//};

int calculate_cost(vector<Vehicle> sensor_reading, double car_s){
    std::cout << "Total length of the sensor cars: " <<sensor_reading.size()<<std::endl;
    for (int i = 0; i< sensor_reading.size(); ++i) {
        if (sensor_reading[i].state == "RF") {
            std::cout << "Right Front car: " <<std::endl;
            std::cout << "s: " <<sensor_reading[i].s<<std::endl;
            std::cout << "dist to our: " <<sensor_reading[i].s - car_s<<std::endl;
            std::cout << "car Lane: " <<sensor_reading[i].lane<<std::endl;
        }
        else if (sensor_reading[i].state == "RB"){
            std::cout << "Right Back car: " <<std::endl;
            std::cout << "s: " <<sensor_reading[i].s<<std::endl;
            std::cout << "dist to our: " <<sensor_reading[i].s - car_s<<std::endl;
            std::cout << "car Lane: " <<sensor_reading[i].lane<<std::endl;
        }
        else if (sensor_reading[i].state == "LF"){
            std::cout << "Left Front car: " <<std::endl;
            std::cout << "s: " <<sensor_reading[i].s<<std::endl;
            std::cout << "dist to our: " <<sensor_reading[i].s - car_s<<std::endl;
            std::cout << "car Lane: " <<sensor_reading[i].lane<<std::endl;
        }
        else if (sensor_reading[i].state == "LB"){
            std::cout << "Left Back car: " <<std::endl;
            std::cout << "s: " <<sensor_reading[i].s<<std::endl;
            std::cout << "dist to our: " <<sensor_reading[i].s - car_s<<std::endl;
            std::cout << "car Lane: " <<sensor_reading[i].lane<<std::endl;
        }
        else {
            std::cout << sensor_reading[i].state <<" car: " <<std::endl;
            std::cout << "s: " <<sensor_reading[i].s<<std::endl;
            std::cout << "dist to our: " <<sensor_reading[i].s - car_s<<std::endl;
            std::cout << "car Lane: " <<sensor_reading[i].lane<<std::endl;
        }
    }
    return 0;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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
    
    int lane = 1;
    double ref_val = 0.0; //MPH
    int ego_state = 0;
    bool slow_down = false;
    bool change_lane_fail = false;

   

    h.onMessage([&change_lane_fail,&slow_down,&ref_val,&lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ego_state]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
                  
    vector<Vehicle> vehicle_list;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

            // Sensor Fusion Data, a list of all other cars on the same side
            //   of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;
            int prev_size = previous_path_x.size();
            bool front_car_detected = false;
            
            

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
//          double dist_inc = 0.5;
//          for (int i = 0; i < 50; ++i) {
//              next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//              next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//           }
            
            // Reading sensor fusion
            if (prev_size >  0) {
                car_s = end_path_s;
            }
            
            for(int i = 0; i < sensor_fusion.size();++i){
                float d = sensor_fusion[i][6];
                // Find the car in the current lane in front of us
                if (d < (2+4*lane+2) && d > (2+4*lane-2) ){
                    // Find out the speed of the car
                    Vehicle car;
                    car.lane = lane;
                    car.s = sensor_fusion[i][5];
                    car.cal_mag_v(sensor_fusion[i][3], sensor_fusion[i][4], (double) prev_size);
                    
                    // Enter 30 meters withn the front car
                    if (car.s > car_s && car.s-car_s <30){
                        front_car_detected = true;
                        car.state = "F";
                        vehicle_list.push_back(car);
                    }
                }
                // Find car in the right, only record cars when there is a right lane and state is not lane keep
                else if (d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) && lane > 0){
                    Vehicle car;
                    car.lane = lane-1;
                    car.s = sensor_fusion[i][5];
                    car.cal_mag_v(sensor_fusion[i][3], sensor_fusion[i][4], (double) prev_size);
                    
                    if (car.s > car_s && car.s - car_s <30){
                        car.state = "RF";
                        vehicle_list.push_back(car);
                    }
                    else if (car.s <= car_s && car_s - car.s <30){
                        car.state = "RB";
                        vehicle_list.push_back(car);
                    }
                }
                
                // Find car in the left lane
                else if (d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) && lane < 2 ){
                    Vehicle car;
                    car.lane = lane+1;
                    car.s = sensor_fusion[i][5];
                    car.cal_mag_v(sensor_fusion[i][3], sensor_fusion[i][4], (double) prev_size);
                    
                    if (car.s > car_s && car.s - car_s <30){
                        car.state = "LF";
                        vehicle_list.push_back(car);
                    }
                    else if (car.s <= car_s && car_s - car.s <30){
                        car.state = "LB";
                        vehicle_list.push_back(car);
                    }
                }
                
            }
           
            // Decision making/jumping states
            switch (ego_state) {
                case 0:
                    if (front_car_detected){
                        // calculate the cost for left and right and decide which one to do
                        int left_shift = calculate_cost(vehicle_list, car_s);
                        
                        
                        // if there is not lane fail detect before, if there is, stay this state.
                        if (!change_lane_fail) {
                            if (left_shift == 1) {//1 left 2 right 0 not assigned
                                // Jump to PLCL
                                ego_state = 1;
                            }
                            else if (left_shift == 2){
                                // Jumpe to PLCR
                                ego_state = 2;
                            }
                            else {
                                ego_state = 0; //stay
                                slow_down = true;
                            }
                        }
                    }
                    
                    break;
                    
                default:
                    break;
            }
            
            // Output of the State machine
            switch (ego_state) {
                // Lane Keeping
                case 0:
                    if (ref_val < 49.5 && !slow_down){
                        ref_val += 0.225;
                    }
                    else if (slow_down && ref_val > 30){
                        ref_val -= 0.225;
                    }
                    else{
                        slow_down = false;
                    }
                    break;
                // Prepare for lane change left
                case 1:
                    std::cout << "Prepare for lane change left" << std::endl;
                    break;
                // Prepare for lane change right
                case 2:
                    std::cout << "Prepare for lane change left" << std::endl;
                    break;
                case 3:
                    lane += 1;
                    ego_state = 0;
                    break;
                case 4:
                    if (ref_val > 29.5){
                        ref_val -= 0.225;
                        break;
                    }
                    else if (!front_car_detected){
                        ego_state = 0;
                    }
                    break;
                    
            }
            
            // You can move this to the N Calculation loop
            
            
            vector<double> ptsx;
            vector<double> ptsy;
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            if(prev_size < 2){
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else{
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];
                
                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);
                
                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
                
            }
            vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            
            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            
            // Transfor the ptsx and y to 0 degree of yaw coordinate
            for (int i = 0; i < ptsx.size();++i){
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }
            tk::spline s;
            
            s.set_points(ptsx, ptsy);
            
            for (int i = 0; i < previous_path_x.size();++i){
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y* target_y);
            
            double x_add_on = 0;
            
            for (int i = 1; i <= 50-previous_path_x.size(); ++i) {
                double N = (target_dist/(0.02*ref_val/2.24)); // Change mph to m/s
                double x_point = x_add_on+target_x/N;
                double y_point = s(x_point);
                
                x_add_on = x_point;
                
                double x_ref = x_point;
                double y_ref = y_point;
                
                //Rotate back to normal
                x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));
                
                x_point += ref_x;
                y_point += ref_y;
                
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
                
            }
            
            


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
