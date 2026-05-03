#include "companion_computer/cc_telemetry.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

TelemetryNode::TelemetryNode() : Node("cc_telemetry"), running_(true) {
    // 1. Initialize State
    memset(&current_state_, 0, sizeof(current_state_));

    // 2. Setup ROS 2 Publishers (Standard messages, so you don't need custom ones yet)
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("telemetry/odom", 10);
    batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("telemetry/battery", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("telemetry/gps", 10);

    // 3. Open UDP Socket (Hardcoded to 14552 for now, can be parameterized later)
    sock_ = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to open UDP socket.");
        exit(EXIT_FAILURE);
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set socket timeout");
    }

    this->declare_parameter<int>("udp_port", 14552); // Default is 14552
    int port;
    this->get_parameter("udp_port", port);
    
    RCLCPP_INFO(this->get_logger(), "Starting node in namespace %s on port %d", 
                this->get_namespace(), port);

    struct sockaddr_in locAddr;
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(port);

    if (bind(sock_, (struct sockaddr *)&locAddr, sizeof(struct sockaddr)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind to UDP 14552");
        close(sock_);
        exit(EXIT_FAILURE);
    }

    RCLCPP_INFO(this->get_logger(), "Listening for MAVLink on UDP 14552...");

    // 4. Launch Background Thread for UDP blocking reads
    udp_thread_ = std::thread(&TelemetryNode::udp_receive_loop, this);

    // 5. Create ROS Timer to publish the state at 50Hz (20ms)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&TelemetryNode::publish_state, this)
    );
}

TelemetryNode::~TelemetryNode() {
    running_ = false;
    shutdown(sock_, SHUT_RDWR); 
    close(sock_);
    if (udp_thread_.joinable()) {
        udp_thread_.join();
    }
}

void TelemetryNode::udp_receive_loop() {
    uint8_t buffer[2048];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (running_ && rclcpp::ok()) {
        ssize_t recsize = recvfrom(sock_, (void *)buffer, sizeof(buffer), 0, NULL, NULL);
        if (recsize > 0) {
            for (ssize_t i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    decode_mavlink(&msg);
                }
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "UDP Loop exiting cleanly.");
}

void TelemetryNode::decode_mavlink(mavlink_message_t* msg) {
    // Lock the mutex so ROS doesn't read half-written data
    std::lock_guard<std::mutex> lock(state_mutex_); 

    switch (msg->msgid) {
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(msg, &pos);
            current_state_.x = pos.x;
            current_state_.y = pos.y;
            current_state_.z = pos.z;
            current_state_.vx = pos.vx;
            current_state_.vy = pos.vy;
            current_state_.vz = pos.vz;
            break;
        }
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t gpos;
            mavlink_msg_global_position_int_decode(msg, &gpos);
            current_state_.alt = gpos.relative_alt / 1000.0f; 
            current_state_.heading = gpos.hdg / 100.0f;       
            break;
        }
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys;
            mavlink_msg_sys_status_decode(msg, &sys);
            current_state_.battery_remaining = sys.battery_remaining;
            current_state_.voltage = sys.voltage_battery / 1000.0f;
            current_state_.current = sys.current_battery / 100.0f;
            break;
        }
        
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
                                           
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(msg, &gps);
            
            printf("DEBUG: Received GPS Packet Fix: %d\n", gps.fix_type);
            printf("DEBUG: satellites visible: %d\n", gps.satellites_visible);

            current_state_.gps_fix = gps.fix_type;
            current_state_.satellites_visible = gps.satellites_visible;
            
            // ADD THESE TWO LINES:
            current_state_.latitude = gps.lat / 10000000.0;
            current_state_.longitude = gps.lon / 10000000.0;
            break;
        }
    }
}

void TelemetryNode::publish_state() {
    // Lock the mutex to read the data safely
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Publish Odometry (Position and Velocity)
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = current_state_.x;
    odom_msg.pose.pose.position.y = current_state_.y;
    odom_msg.pose.pose.position.z = current_state_.z;
    odom_msg.twist.twist.linear.x = current_state_.vx;
    odom_msg.twist.twist.linear.y = current_state_.vy;
    odom_msg.twist.twist.linear.z = current_state_.vz;
    odom_pub_->publish(odom_msg);

    // Publish Battery
    auto batt_msg = sensor_msgs::msg::BatteryState();
    batt_msg.header.stamp = this->now();
    batt_msg.percentage = current_state_.battery_remaining / 100.0f;
    batt_msg.voltage = current_state_.voltage;
    batt_msg.current = current_state_.current;
    batt_msg.present = true;
    batt_pub_->publish(batt_msg);

    auto gps_msg = sensor_msgs::msg::NavSatFix();
    gps_msg.header.stamp = this->now();
    gps_msg.header.frame_id = "gps";
    gps_msg.latitude = current_state_.latitude;
    gps_msg.longitude = current_state_.longitude;
    
    if (current_state_.gps_fix >= 2) {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    } else {
        gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    }
    gps_msg.status.service = static_cast<uint16_t>(current_state_.satellites_visible);
    gps_pub_->publish(gps_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TelemetryNode>());
    rclcpp::shutdown();
    return 0;
}
