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

		  // Convert to Eigen
		  // Pointer to first element
		  double* ptr_x = &ptsx[0];
		  double* ptr_y = &ptsy[0];

		 // cout << "ptr_x : " << &ptsx[0] << endl;

		  Eigen::Map<Eigen::VectorXd> ptsx_vector(ptr_x, ptsx.size());
		  Eigen::Map<Eigen::VectorXd> ptsy_vector(ptr_y, ptsy.size());

		  double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

		  vector<double> transformed_pts_x;
		  vector<double> transformed_pts_y;

		 // cout << "To local points : " << endl;
		  utils.to_local_points(ptsx, ptsy, px, py, psi, transformed_pts_x, transformed_pts_y);

		 // cout << "Transformed x points : " << transformed_pts_x.size() << endl;
		 // cout << "Transformed y points : " << transformed_pts_y.size() << endl;

		  double* ptr_transformed_x = &transformed_pts_x[0];
		  double* ptr_transformed_y = &transformed_pts_y[0];

		  //cout << "ptr_transformed_x : " << &transformed_pts_x[0] << endl;

		  Eigen::Map<Eigen::VectorXd> transformed_ptsx_vector(ptr_transformed_x, transformed_pts_x.size());
		  Eigen::Map<Eigen::VectorXd> transformed_ptsy_vector(ptr_transformed_y, transformed_pts_y.size());
		  
		  //cout << "Polyfit " << endl;
		  auto coeffs = utils.polyfit(transformed_ptsx_vector, transformed_ptsy_vector, 3);

		  //cout << "Polyeval: " << endl;
		  double cte = utils.polyeval(coeffs, 0);
		  double epsi = -atan(coeffs[1]);

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
		  Eigen::VectorXd state(6);
		  //state << px, py, psi, v, cte, epsi;
		  state << 0.0, 0.0, 0.0, v, cte, epsi;
		  //Eigen::VectorXd actuators(2);
		  //actuators << deg2rad(5), 1;
		  
		  //state = utils.globalKinematic(state, actuators, 0.3, Lf);

		  auto mpcs = mpc.Solve(state, coeffs);

		  // mpcs order of retruned values
		  // 0: steer_value, 
		  // 1: throttle_value, 
		  // 2: mpc_x_vals, 
		  // 3: mpc_y_vals 

          double steer_value;
          double throttle_value;
		  
		  steer_value = mpcs[0] / (deg2rad(25) * Lf);
		  throttle_value = mpcs[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

		  for (int i = 2; i < mpcs.size(); i++)
		  {
			  if (i % 2 == 0)
				  mpc_x_vals.push_back(mpcs[i]);
			  else
				  mpc_y_vals.push_back(mpcs[i]);
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
		  vector<double> next_x_vals;
		  vector<double> next_y_vals;
		  int Nex = 30;
		  for (int i = 0; i < Nex; ++i) {
			  next_x_vals.push_back(i * 2);
			  next_y_vals.push_back(utils.polyeval(coeffs, i * 2));
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
