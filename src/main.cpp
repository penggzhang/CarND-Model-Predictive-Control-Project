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
#include <tuple>

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

// Transform coordinates
std::tuple<vector<double>, vector<double>> transformCoord(vector<double> ptsx, vector<double> ptsy, double px, double py, double psi) {
    vector<double> ptsx_veh;
    vector<double> ptsy_veh;
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - px;
        double shift_y = ptsy[i] - py;
        ptsx_veh.push_back(shift_x * cos(psi) + shift_y * sin(psi));
        ptsy_veh.push_back(shift_x * (-sin(psi)) + shift_y * cos(psi));
    }
    return std::make_tuple(ptsx_veh, ptsy_veh);
}

// Define latency (second) and the vehicle length from front to CoG (m)
double latency = 0.1;
const double Lf = 2.67;

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
                    
                    // Set start timer to measure latency duration
                    auto clock_start = chrono::system_clock::now();
                    
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];
                    
                    // For computing state after latency
                    double steer_angle = j[1]["steering_angle"];
                    // Transform from values in [-1, 1] to radians,
                    // also take into account the vehicle length
                    double delta = steer_angle * deg2rad(25) * Lf;
                    // Take throttle values as the estimator for acceleration
                    double a = j[1]["throttle"];
                    
                    /*
                     * TODO: Calculate steering angle and throttle using MPC.
                     *
                     * Both are in between [-1, 1].
                     *
                     */
                    
                    // Predict the state after latency
                    px += v * cos(psi) * latency;
                    py += v * sin(psi) * latency;
                    psi -= v / Lf * delta * latency; // Clockwise steering corresponds to positive delta
                    v += a * latency;
                    
                    // Transform waypoints (ptsx[i], ptsy[i]) from map coordinates
                    // to vehicle coordinates (ptsx_veh[i], ptsy_veh[i])
                    vector<double> ptsx_veh, ptsy_veh;
                    std::tie(ptsx_veh, ptsy_veh) = transformCoord(ptsx, ptsy, px, py, psi);
                    // Cast std::vector to Eigen::VectorXd
                    double * ptr_vx = &ptsx_veh[0];
                    double * ptr_vy = &ptsy_veh[0];
                    int n_pts = ptsx.size();
                    Eigen::Map<Eigen::VectorXd> v_ptsx_veh(ptr_vx, n_pts);
                    Eigen::Map<Eigen::VectorXd> v_ptsy_veh(ptr_vy, n_pts);
                    
                    // Fit a polynomial to the transformed waypoints
                    auto coeffs = polyfit(v_ptsx_veh, v_ptsy_veh, 3);
                    
                    // Find cte and orientation error at the state after latency
                    double cte, epsi;
                    // In vehicle coordinates, the fitted polynomial at x = 0 corresponds to the cte value.
                    cte = polyeval(coeffs, 0);
                    // Calculate the derivative of the fitted polynomial at x = 0.
                    // Its arctan subtracted from psi(=0) gives the orientation error,
                    // i.e. epsi = psi - atan(coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * pow(x, 2)).
                    epsi = - atan(coeffs[1]);
                    
                    // The state after latency in vehicle coordinates
                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;
                    
                    // Apply the Solve method of MPC to calculate the actuations after the latency.
                    auto result = mpc.Solve(state, coeffs);
                    double steer_value = result[0];
                    double throttle_value = result[1];
                    
                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value / (deg2rad(25) * Lf);
                    msgJson["throttle"] = throttle_value;
                    
                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (int i = 2; i < result.size() - 2; i++) { // Drop the last MPC point
                        if (i % 2 == 0) {
                            mpc_x_vals.push_back(result[i]);
                        } else {
                            mpc_y_vals.push_back(result[i]);
                        }
                    }
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;
                    
                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    double x_step = 2.5;
                    int n_pts_ref = 25;
                    for (int i = 1; i < n_pts_ref; i++) {
                        double x_val = x_step * i;
                        next_x_vals.push_back(x_val);
                        next_y_vals.push_back(polyeval(coeffs, x_val));
                    }
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does not actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(140));
                    
                    // Measure latency duration
                    auto clock_stop = chrono::system_clock::now();
                    chrono::duration<double, std::milli> timetaken = clock_stop - clock_start;
                    std::cout << "Latency duration (milli second): " << timetaken.count() << std::endl;
                    
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
