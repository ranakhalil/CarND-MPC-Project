#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "Utils.h"

// for convenience
using json = nlohmann::json;

// Utils is initialized here!
Utils utils;

const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  
  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
		  double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

		  vector<double> transform_x;
		  vector<double> transform_y;

		 // cout << "To local points : " << endl;
		  utils.to_local_points(ptsx, ptsy, px, py, psi, transform_x, transform_y);

		  // Convert to Eigen
		  // Pointer to first element
		  double* ptr_x = &transform_x[0];
		  double* ptr_y = &transform_y[0];

		  // cout << "ptr_x : " << &ptsx[0] << endl;

		  Eigen::Map<Eigen::VectorXd> v_ptsx(ptr_x, transform_x.size());
		  Eigen::Map<Eigen::VectorXd> v_ptsy(ptr_y, transform_y.size());
		  
		  //cout << "Polyfit " << endl;
		  Eigen::VectorXd coeffs = utils.polyfit(v_ptsx, v_ptsy, 3);

		  //cout << "Polyeval: " << endl;
		  double cte = utils.polyeval(coeffs, 0);
		  double epsi = -CppAD::atan(coeffs[1]);

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  double delta_t = 0.43704;
		  // 100 ms
		  double latency = 0.1;
		  
		  Eigen::VectorXd state(6);
		  // Recall the equations for the model:
		  // Global Kinematic
		  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		  // v_[t+1] = v[t] + a[t] * dt
		  state << 0.0 + v * CppAD::cos(mpc.steering_) * delta_t * latency,
					0.0 + v * CppAD::sin(mpc.steering_) * delta_t * latency,
					0.0,
					0.0 + v * delta_t, 
					cte, 
					epsi;

		  auto mpcs = mpc.Solve(state, coeffs);

		  // mpcs order of retruned values
		  // 0: steer_value, 
		  // 1: throttle_value, 
		  // 2: mpc_x_vals, 
		  // 3: mpc_y_vals 

          auto steer_value = -mpc.steering_ / deg2rad(25);
          auto throttle_value = mpc.a_;
		  
		  steer_value = steer_value;
		  throttle_value = throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value * 1.5;

          //Display the MPC predicted trajectory 
         // vector<double> mpc_x_vals;
         // vector<double> mpc_y_vals;
		  /*
		  for (int i = 2; i < mpcs.size(); i++)
		  {
			  if (i % 2 == 0)
				  mpc_x_vals.push_back(mpcs[i]);
			  else
				  mpc_y_vals.push_back(mpcs[i]);
		  }
		  */

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc.predicted_x_vals;
          msgJson["mpc_y"] = mpc.predicted_y_vals;

          //Display the waypoints/reference line
		  vector<double> next_x_vals;
		  next_x_vals.resize(transform_x.size());
		  vector<double> next_y_vals;
		  next_y_vals.resize(transform_y.size());

		  for (int p = 0; p < transform_x.size(); p++)
		  {
			  next_x_vals[p] = transform_x[p];
			  next_y_vals[p] = transform_y[p];
		  }
		 

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

		  msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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
