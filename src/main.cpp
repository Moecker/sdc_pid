#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "PID.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
    return 3.14159265359;
}
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


double Run(double cte)
{
    return cte;
}


PID Twiddle(double current_cte, double tolerance = 0.2, int max_iterations = 1)
{
    PID pid;

    std::vector<double> taus = {0.0, 0.0, 0.0};
    std::vector<double> errors = {1.0, 1.0, 1.0};

    // Somehow run the program here
    auto best_cte = Run(current_cte);
    auto iteration = 0;
    while (pid.TotalError() > tolerance)
    {
        std::cout << "Iteration " << iteration << ":  Best error = " << best_cte;
        for (int i = 0; i < taus.size(); ++i)
        {
            taus[i] += errors[i];

            auto cte = Run(current_cte);
            pid.UpdateError(cte);

            if (cte < best_cte)
            {
                best_cte = cte;
                errors[i] *= 1.1;
            }
            else
            {
                taus[i] -= 2 * errors[i];

                auto cte = Run(current_cte);
                pid.UpdateError(cte);

                if (cte < best_cte)
                {

                    best_cte = cte;
                    errors[i] *= 1.1;
                }
                else
                {
                    taus[i] += errors[i];
                    errors[i] *= 0.9;
                }
            }
        }
    }
    iteration += 1;
    return pid;
}

int main()
{
    uWS::Hub hub;

    PID pid;
    const auto p = 0.06;
    const auto i = 0.0005;
    const auto d = 0.8;

    pid.Init(p, i, d);

    hub.onMessage([&pid](uWS::WebSocket<uWS::SERVER> web_socket, char* data, size_t length, uWS::OpCode op_code) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "")
            {
                auto json_data = json::parse(s);
                std::string event = json_data[0].get<std::string>();
                if (event == "telemetry")
                {
                    // json_data[1] is the data JSON object
                    double cte = std::stod(json_data[1]["cte"].get<std::string>());
                    double speed = std::stod(json_data[1]["speed"].get<std::string>());
                    double angle = std::stod(json_data[1]["steering_angle"].get<std::string>());

                    // Calculate steering value here, remember the steering value is [-1, 1].
                    double steer_value =
                        -pid.tau_p_ * pid.p_error_ - pid.tau_d_ * pid.d_error_ - pid.tau_i_ * pid.i_error_;

                    // Debug output
                    std::cout << "CTE: " << cte << "Speed: " << speed << "Angle: " << angle << " Steering Value: " << steer_value << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";

                    web_socket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    pid.UpdateError(cte);

                    // pid = Twiddle(cte);
                }
            }
            else
            {
                // Manual driving
                std::string message = "42[\"manual\",{}]";
                web_socket.send(message.data(), message.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    hub.onHttpRequest([](uWS::HttpResponse* response, uWS::HttpRequest request, char* data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (request.getUrl().valueLength == 1)
        {
            response->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            response->end(nullptr, 0);
        }
    });

    hub.onConnection(
        [&hub, &pid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { pid.UpdateError(0.0); std::cout << "Connected!!!" << std::endl; });

    hub.onDisconnection([&hub](uWS::WebSocket<uWS::SERVER> web_socket, int code, char* message, size_t length) {
        web_socket.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (hub.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    hub.run();
}
