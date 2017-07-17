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

// for convenience
using json = nlohmann::json;

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  // This is to enforce actuation continuity between 2 invocations of the solver
  double acc_last_value = 0.0;
  double delta_last_value = 0.0;

  h.onMessage([&mpc, &acc_last_value, &delta_last_value]
                  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          auto start = std::chrono::system_clock::now();

          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = - (double) j[1]["psi"];
          double v = (double) j[1]["speed"];

          while (psi >= M_PI) psi -= 2*M_PI;
          while (psi < -M_PI) psi += 2*M_PI;
          //cout<<"px:"<<px<<" py:"<<py<<" psi:"<<psi<<" v:"<<v<<endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */


          // Transform waypoints in map coordinates to vehicle coordinates.
          Eigen::VectorXd xval(ptsx.size());
          Eigen::VectorXd yval(ptsy.size());

          for (size_t i = 0; i < ptsx.size() && i<ptsy.size(); ++i) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            double psi_local = psi;
            xval(i) = x * cos(psi_local) - y *sin(psi_local) ;
            yval(i) = x * sin(psi_local) + y *cos(psi_local) ;
          }
          double x_vehicle = 0.0;
          double y_vehicle = 0.0;
          //psi = -psi;
          // fit a polynomial to the above x and y coordinates
          auto coeffs = polyfit(xval, yval, 3) ;

          // calculate the current cross track error
          //  It is calculated using the track as a reference
          double cte = polyeval(coeffs,x_vehicle) - y_vehicle;

          // Calculate the initial orientation error for the objective function.
          // The objective orientation is given by : psi = atan(f'(x))
          // Here f(x) is a 2nd order poly
          // f'(x) = 2*coeffs[2] + coeffs[1]
          double Df_y = coeffs[1];
          double x_power_prime = xval(0);
          for(int deg=2; deg<coeffs.size(); ++deg){
            Df_y += deg * coeffs[deg] * x_power_prime;
            x_power_prime = x_power_prime * xval(0);
          }

          //psi = -psi;
          double epsi =  (atan(Df_y) - psi);

          while (epsi >= M_PI) epsi -= 2*M_PI;
          while (epsi < -M_PI) epsi += 2*M_PI;

          Eigen::VectorXd state(6);
          state << x_vehicle, y_vehicle, 0.0, v, cte, epsi;


          /* do some work */
          auto solution_x = mpc.Solve(state, coeffs, acc_last_value,
                                delta_last_value);

          auto end = std::chrono::system_clock::now();
          auto elapsed = (std::chrono::duration_cast<std::chrono::microseconds>) (end - start);
          std::cout <<"elapsed.count:"<< elapsed.count()/1000 << " ms \n";


          double delay_s = 0.100 + ((double)(elapsed.count())*1e-6);
          const size_t delay_samples = (size_t) (delay_s /(MPC::dt));
          if(delay_samples > MPC::N ){
            std::cerr<<"The delay is larger than the model predictive horizon"<<std::endl;
          }


          std::cout << "x = " << solution_x[x_start] <<"; delay_s ="<< delay_s;
          std::cout << "; y = " << solution_x[y_start];
          std::cout << "; psi = " << solution_x[psi_start];
          std::cout << "; v = " << solution_x[v_start] << std::endl;
          std::cout << "cte = " << solution_x[cte_start];
          std::cout << "; epsi = " << solution_x[epsi_start];
          std::cout << "; delta = " << solution_x[delta_start];
          std::cout << "; a = " << solution_x[a_start] << std::endl;
          delta_last_value = solution_x[delta_start];
          acc_last_value   = solution_x[a_start];

          double steer_value = -solution_x[delta_start+delay_samples];
          double throttle_value = solution_x[a_start+delay_samples];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          if(fabs(solution_x[delta_start + delay_samples*2]/deg2rad(25))>0.1 ||
            fabs(solution_x[delta_start + delay_samples*4]/deg2rad(25))>0.085 ||
              fabs(solution_x[delta_start + delay_samples*6]/deg2rad(25))>0.044){
            ref_v = 60;
          } else{
            ref_v = 110;
          }
          std::cout <<"ref_v:" << ref_v << std::endl;


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          mpc_x_vals.clear();
          mpc_y_vals.clear();
          for (size_t k = delay_samples; k < MPC::N; ++k) {
            mpc_x_vals.push_back(solution_x[x_start + k]);
            mpc_y_vals.push_back(solution_x[y_start + k]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          next_x_vals.clear();
          next_y_vals.clear();

          for (size_t k = 0; k < xval.size() && k<yval.size(); ++k) {
            double x = xval(k) ; //solution_x[k];
            double fy = polyeval(coeffs,x);
            next_x_vals.push_back(x);
            next_y_vals.push_back(fy);
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
