#pragma once
#include <stdint.h>
#include <mutex>
#include <thread>

#include "c_library_v2/common/mavlink.h"



// Standard ROS 2 Includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Your internal C++ struct to hold the shattered MAVLink data
struct DroneState {
    float x, y, z;
    float vx, vy, vz;
    float alt;
    float heading;
    int gps_fix;
    int satellites_visible;
    int battery_remaining;

    float voltage;
    float current;

    double latitude;
    double longitude;
};

class TelemetryNode : public rclcpp::Node {
public:
    TelemetryNode();
    ~TelemetryNode();

private:
    // ROS 2 Publishers to broadcast state to cc_defender_guidance
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;


    // UDP Networking
    int sock_;
    std::thread udp_thread_;
    bool running_;

    // State Management
    DroneState current_state_;
    std::mutex state_mutex_; // Prevents ROS and UDP from colliding in memory

    // Core functions
    void udp_receive_loop();
    void publish_state();
    void decode_mavlink(mavlink_message_t* msg);
};
