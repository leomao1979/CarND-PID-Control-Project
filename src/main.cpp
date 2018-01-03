#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"
#include <math.h>
#include <string.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main(int argc, char *argv[])
{
    uWS::Hub h;
    // Initialize the pid variable.
    Twiddle twiddle;
    PID pid;
    bool twiddle_mode = false;
    if (argc > 1) {
        if (strcmp(argv[1], "twiddle") == 0) {
            twiddle_mode = true;
        }
    }
    std::cout << "Running mode: ";
    if (twiddle_mode) {
        std::cout << "twiddle" << std::endl;
    } else {
        std::cout << "pid" << std::endl;
        double Kp = 0.2;
        double Ki = 0.0082805;
        double Kd = 4.18;
        if (argc == 4) {
            Kp = atof(argv[1]);
            Ki = atof(argv[2]);
            Kd = atof(argv[3]);
        }
        std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;
        pid.Init(Kp, Ki, Kd);
    }
    
    h.onMessage([&twiddle, &pid, &twiddle_mode](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    
                    bool send_steer_value = true;
                    if (twiddle_mode) {
                        twiddle.Update(cte, speed);
                        if (twiddle.ShallReset()) {
                            // Reset
                            std::string msg = "42[\"reset\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            send_steer_value = false;
                        }
                    } else {
                        pid.UpdateError(cte);
                    }
                    if (send_steer_value) {
                        double steer_value;
                        /*
                         * Calcuate steering value here, remember the steering value is [-1, 1].
                         * NOTE: Feel free to play around with the throttle and speed. Maybe use
                         * another PID controller to control the speed!
                         */
                        if (twiddle_mode) {
                            steer_value = -twiddle.TotalError();
                        } else {
                            steer_value = -pid.TotalError();
                        }
                        if (steer_value < -1) {
                            steer_value = -1;
                        }
                        if (steer_value > 1) {
                            steer_value = 1;
                        }
                        json msgJson;
                        msgJson["steering_angle"] = steer_value;
                        msgJson["throttle"] = 0.3;
//                        msgJson["throttle"] = 1 - 0.5 * fabs(steer_value);
                        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                        if (!twiddle_mode) {
                            // DEBUG
                            std::cout << "CTE: " << cte << ", Speed: " << speed << ", Angle: " << angle << ". Steering Value: " << steer_value << std::endl;
                            std::cout << msg << std::endl;
                        }
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    }
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h, &twiddle, &twiddle_mode](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << std::endl;
        std::cout << "Connected!!!" << std::endl;
        if (twiddle_mode) {
            twiddle.Start();
        }
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
