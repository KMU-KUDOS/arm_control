#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <sstream>
#include <iomanip>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>
#include <mutex>
#include <algorithm>
#include <cmath>

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

class RMD_COMMAND {
public:
    void POSITION_CONTROL(int s, int ID, int32_t angleControl, uint16_t maxSpeed) {
        struct can_frame frame;
        uint8_t maxSpeed_bytes[2];
        std::memcpy(maxSpeed_bytes, &maxSpeed, 2);
        uint8_t angleControl_bytes[4];
        std::memcpy(angleControl_bytes, &angleControl, 4);

        frame.can_id = 0x140 + ID;
        frame.can_dlc = 8;
        frame.data[0] = 0xA4;
        frame.data[1] = 0x00;
        frame.data[2] = maxSpeed_bytes[0];
        frame.data[3] = maxSpeed_bytes[1];
        frame.data[4] = angleControl_bytes[0];
        frame.data[5] = angleControl_bytes[1];
        frame.data[6] = angleControl_bytes[2];
        frame.data[7] = angleControl_bytes[3];

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write failed");
        }
    }

    void SEND_RAW_FRAME(int s, uint32_t can_id, const std::vector<uint8_t>& data) {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = data.size();

        if (data.size() > CAN_MAX_DLEN) {
            RCLCPP_ERROR(rclcpp::get_logger("rmd_command"), "Data size exceeds CAN_MAX_DLEN");
            return;
        }
        std::memcpy(frame.data, data.data(), data.size());

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Raw frame write failed");
        } else {
            // Log specific frames for debugging (e.g., position control commands)
            bool should_log = false;
            if (data.size() >= 2) {
                if (data[0] == 0xA4 && data[1] != 0x00) {
                    should_log = true;
                }
            }

            if (should_log) {
                std::stringstream ss;
                ss << "CAN ID: " << std::hex << std::setw(3) << std::setfill('0') << can_id << " Data: ";
                for (uint8_t byte : data) {
                    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
                }
                RCLCPP_INFO(rclcpp::get_logger("rmd_command"), "%s", ss.str().c_str());
            }
        }
    }
};

class JoystickToRMDControl : public rclcpp::Node {
public:
    JoystickToRMDControl() : Node("joystick_to_rmd_control") {
        // Declare parameters
        this->declare_parameter<int>("axis_can10_motor1", 3);
        this->declare_parameter<int>("axis_can10_motor2", 1);
        this->declare_parameter<int>("axis_can11_motor1", 4);
        this->declare_parameter<int>("axis_can13_motor1", 6);
        this->declare_parameter<std::vector<int>>("axis_can13_motor2_combined", {2, 5});
        this->declare_parameter<int>("axis_can12_motor1", 0);
        this->declare_parameter<int>("axis_can12_motor2", 7);

        this->declare_parameter<bool>("invert_can10_motor1", false);
        this->declare_parameter<bool>("invert_can10_motor2", false);
        this->declare_parameter<bool>("invert_can11_motor1", false);
        this->declare_parameter<bool>("invert_can11_motor2", true);
        this->declare_parameter<bool>("invert_can13_motor1", false);
        this->declare_parameter<bool>("invert_can13_motor2", false);
        this->declare_parameter<bool>("invert_can12_motor1", false);
        this->declare_parameter<bool>("invert_can12_motor2", false);

        this->declare_parameter<double>("cos_acceleration_scale", 1.0); // Parameter for acceleration scaling
        this->declare_parameter<double>("cos_curve_factor", 1.0);     // Parameter for 1-cos curve factor
        this->declare_parameter<double>("curve_exponent", 2.0);        // Parameter for power function curve


        // Get parameter values
        axis_can10_motor1_ = this->get_parameter("axis_can10_motor1").as_int();
        axis_can10_motor2_ = this->get_parameter("axis_can10_motor2").as_int();
        axis_can11_motor1_ = this->get_parameter("axis_can11_motor1").as_int();
        axis_can13_motor1_ = this->get_parameter("axis_can13_motor1").as_int();
        axis_can12_motor1_ = this->get_parameter("axis_can12_motor1").as_int();
        axis_can12_motor2_ = this->get_parameter("axis_can12_motor2").as_int();

        std::vector<int> combined_axes_default = {2, 5};
        auto combined_axes = this->get_parameter("axis_can13_motor2_combined").as_integer_array();
        axis_can13_motor2_combined_ = combined_axes.empty() ? combined_axes_default : std::vector<int>(combined_axes.begin(), combined_axes.end());

        invert_can10_motor1_ = this->get_parameter("invert_can10_motor1").as_bool();
        invert_can10_motor2_ = this->get_parameter("invert_can10_motor2").as_bool();
        invert_can11_motor1_ = this->get_parameter("invert_can11_motor1").as_bool();
        invert_can11_motor2_ = this->get_parameter("invert_can11_motor2").as_bool();
        invert_can13_motor1_ = this->get_parameter("invert_can13_motor1").as_bool();
        invert_can13_motor2_ = this->get_parameter("invert_can13_motor2").as_bool();
        invert_can12_motor1_ = this->get_parameter("invert_can12_motor1").as_bool();
        invert_can12_motor2_ = this->get_parameter("invert_can12_motor2").as_bool();

        cos_acceleration_scale_ = this->get_parameter("cos_acceleration_scale").as_double();
        cos_curve_factor_ = this->get_parameter("cos_curve_factor").as_double();
        curve_exponent_ = this->get_parameter("curve_exponent").as_double();


        // Subscribe to joystick messages
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(1).best_effort(), std::bind(&JoystickToRMDControl::joy_callback, this, std::placeholders::_1));

        //can_socket_10_ = open_can_socket("can10");
        //can_socket_11_ = open_can_socket("can11");
        can_socket_12_ = open_can_socket("can12");
        //can_socket_13_ = open_can_socket("can13");

        if (can_sockets_ok()) {
            RCLCPP_INFO(this->get_logger(), "CAN sockets opened successfully.");
        } else {
            RCLCPP_FATAL(this->get_logger(), "Failed to open one or more CAN sockets. Exiting...");
            rclcpp::shutdown();
            return;
        }

        last_command_time_ = std::chrono::steady_clock::now();
        last_angleControl_motors_.resize(8, 0);
        target_angleControl_motors_.resize(8, 0);


        RMD_COMMAND rmd;

        send_startup_raw_frames(rmd);
        RCLCPP_INFO(this->get_logger(), "Raw CAN frames sent at startup for initial position.");

        timer_ = this->create_wall_timer(
            command_interval_,
            std::bind(&JoystickToRMDControl::timer_callback, this));

        last_callback_start_time_ = std::chrono::steady_clock::now();

        // Initialize previous joystick values
        prev_joy_input_can10_motor1_ = 0.0;
        prev_joy_input_can10_motor2_ = 0.0;
        prev_joy_input_can11_motor1_ = 0.0;
        prev_joy_input_can11_motor2_ = 0.0; // can11 motor2
        
    }

    ~JoystickToRMDControl() {
        close_can_sockets();
    }

private:
    bool can_sockets_ok() const {
        return can_socket_10_ >= 0 && can_socket_11_ >= 0 && can_socket_12_ >= 0 && can_socket_13_ >= 0;
    }

    void close_can_sockets() {
        if (can_socket_10_ >= 0) close(can_socket_10_);
        if (can_socket_11_ >= 0) close(can_socket_11_);
        if (can_socket_12_ >= 0) close(can_socket_12_);
        if (can_socket_13_ >= 0) close(can_socket_13_);
    }

    int open_can_socket(const char *interface_name) {
        int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd < 0) {
            perror("Socket creation failed");
            return -1;
        }
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
        if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
            perror("ioctl SIOCGIFINDEX failed");
            close(socket_fd);
            return -1;
        }
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Socket bind failed");
            close(socket_fd);
            return -1;
        }
        return socket_fd;
    }

    void send_startup_raw_frames(RMD_COMMAND& rmd) {
        uint32_t raw_can_id1 = 0x141;
        uint32_t raw_can_id2 = 0x142;
        std::vector<uint8_t> raw_can_data_initial_pos = {0xA4, 0x00, 0x96, 0x02, 0x00, 0x00, 0x00, 0x00};

        if (can_sockets_ok()) {
            rmd.SEND_RAW_FRAME(can_socket_10_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_10_, raw_can_id2, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_11_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_11_, raw_can_id2, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_12_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_12_, raw_can_id2, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_13_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_13_, raw_can_id2, raw_can_data_initial_pos);

            std::this_thread::sleep_for(5s);
        }
    }

private:
    std::vector<int32_t> last_angleControl_motors_;
    std::vector<int32_t> target_angleControl_motors_;
    std::chrono::steady_clock::time_point last_command_time_;
    const std::chrono::milliseconds command_interval_ = 10ms;
    std::chrono::steady_clock::time_point last_callback_start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy::SharedPtr latest_joy_msg_;
    std::mutex joy_msg_mutex_;

    const uint16_t max_speed_scale_ = 240;
    const uint16_t min_max_speed_ = 0;
    const float max_angle_scale_ = 37700.0;
    const float min_angle_scale_ = 0;

    int axis_can10_motor1_;
    int axis_can10_motor2_;
    int axis_can11_motor1_;
    int axis_can13_motor1_;
    std::vector<int> axis_can13_motor2_combined_;
    int axis_can12_motor1_;
    int axis_can12_motor2_;

    bool invert_can10_motor1_;
    bool invert_can10_motor2_;
    bool invert_can11_motor1_;
    bool invert_can11_motor2_;
    bool invert_can13_motor1_;
    bool invert_can13_motor2_;
    bool invert_can12_motor1_;
    bool invert_can12_motor2_;

    double cos_acceleration_scale_;
    double cos_curve_factor_;
    double curve_exponent_;

    // Variables for smooth deceleration
    float prev_joy_input_can10_motor1_;
    float prev_joy_input_can10_motor2_;
    float prev_joy_input_can11_motor1_;
    float prev_joy_input_can11_motor2_;
    const std::chrono::milliseconds deceleration_time_ = 500ms; // 0.5 seconds


    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joy_msg_mutex_);
        latest_joy_msg_ = msg;
    }
     uint16_t calculate_max_speed(float joystick_input) {
        if (std::abs(joystick_input) <= 0.1) {
            return min_max_speed_;
        }
        uint16_t calculated_speed = static_cast<uint16_t>(std::abs(joystick_input) * max_speed_scale_);
        return std::clamp(calculated_speed, min_max_speed_, max_speed_scale_);
    }
      int32_t calculate_angle_increment_with_cos(float joy_input, float dynamic_angle_scale, double period_sec) {
        if (std::abs(joy_input) <= 0.05) {
            return 0;
        }
        float normalized_input = std::abs(joy_input);
        float modified_normalized_input = std::pow(normalized_input, curve_exponent_); // Power function 적용
        float cos_value = (1.0 - cos(modified_normalized_input * M_PI * cos_curve_factor_)) / 2.0; // Apply curve factor and power input
        return static_cast<int32_t>(joy_input * dynamic_angle_scale * period_sec * cos_value * cos_acceleration_scale_); // Apply acceleration scale
    }

    float smooth_decelerate(float current_input, float &prev_input, double elapsed_time_ms) {
       if (std::abs(current_input) > 0.05) { // Keep updating prev_input if joystick is active
            prev_input = current_input;
            return current_input;
        }

        if (std::abs(prev_input) <= 0.0001) {  // Use a small threshold instead of direct comparison to zero
            return 0.0;
        }


        // Linear deceleration
        float deceleration_rate = std::abs(prev_input) / (deceleration_time_.count()); // Decrease per millisecond
        float delta = deceleration_rate * elapsed_time_ms;

        if (prev_input > 0) {
            prev_input -= delta;
            prev_input = std::max(0.0f, prev_input); // Ensure it doesn't go below zero
        } else {
            prev_input += delta;
            prev_input = std::min(0.0f, prev_input); // Ensure it doesn't go above zero
        }

      return prev_input;
    }
     void timer_callback() {
        auto current_callback_start_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> period = current_callback_start_time - last_callback_start_time_;
        last_callback_start_time_ = current_callback_start_time;
        auto period_sec = period.count();

        RCLCPP_DEBUG(this->get_logger(), "Expected period: %ld ms, Actual period: %ld ms", command_interval_.count(), static_cast<long int>(period_sec * 1000));

        RMD_COMMAND rmd;
        float dynamic_angle_scale;
        float joy_input_can10_motor1, joy_input_can10_motor2, joy_input_can11_motor1, joy_input_can11_motor2; // can11 motor2추가


        auto now = std::chrono::steady_clock::now();
        if (now - last_command_time_ >= command_interval_) {
            sensor_msgs::msg::Joy::SharedPtr msg;
            {
                std::lock_guard<std::mutex> lock(joy_msg_mutex_);
                msg = latest_joy_msg_;
            }
              if (msg) {
                if (true) {
                    RCLCPP_DEBUG(this->get_logger(), "Raw Joystick Axes: A3=%.2f, A1=%.2f, A4=%.2f, A6=%.2f, A2=%.2f, A5=%.2f, A0=%.2f, A7=%.2f",
                                 msg->axes[3], msg->axes[1], msg->axes[4], msg->axes[6], msg->axes[2], msg->axes[5], msg->axes[0], msg->axes[7]);


                    // CAN10 - Motor 1 (1-cos 적용 및 감속)
                    joy_input_can10_motor1 = msg->axes[axis_can10_motor1_];
                    if (invert_can10_motor1_) joy_input_can10_motor1 *= -1.0;
                    joy_input_can10_motor1 = smooth_decelerate(joy_input_can10_motor1, prev_joy_input_can10_motor1_, command_interval_.count());
                    uint16_t maxSpeed_can10_motor1 = calculate_max_speed(joy_input_can10_motor1);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can10_motor1));
                    int32_t angle_increment_can10_motor1 = calculate_angle_increment_with_cos(joy_input_can10_motor1, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[0] += angle_increment_can10_motor1;
                    RCLCPP_DEBUG(this->get_logger(), "Target Angle CAN10 Motor 1: %d, Increment: %d", target_angleControl_motors_[0] / 3600, angle_increment_can10_motor1);


                    // CAN10 - Motor 2 (1-cos 적용 및 감속)
                    joy_input_can10_motor2 = msg->axes[axis_can10_motor2_];
                    if (invert_can10_motor2_) joy_input_can10_motor2 *= -1.0;
                    joy_input_can10_motor2 = smooth_decelerate(joy_input_can10_motor2, prev_joy_input_can10_motor2_, command_interval_.count());
                    uint16_t maxSpeed_can10_motor2 = calculate_max_speed(joy_input_can10_motor2);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can10_motor2));
                    int32_t angle_increment_can10_motor2 = calculate_angle_increment_with_cos(joy_input_can10_motor2, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[1] += angle_increment_can10_motor2;


                    // CAN11 - Motor 1 (1-cos 적용 및 감속)
                    joy_input_can11_motor1 = msg->axes[axis_can11_motor1_];
                    if (invert_can11_motor1_) joy_input_can11_motor1 *= -1.0;
                    joy_input_can11_motor1 = smooth_decelerate(joy_input_can11_motor1, prev_joy_input_can11_motor1_, command_interval_.count());
                    uint16_t maxSpeed_can11_motor1 = calculate_max_speed(joy_input_can11_motor1);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can11_motor1));
                    int32_t angle_increment_can11_motor1 = calculate_angle_increment_with_cos(joy_input_can11_motor1, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[2] += angle_increment_can11_motor1;


                    // CAN11 - Motor 2 (opposite of motor 1)
                    joy_input_can11_motor2 = -joy_input_can11_motor1;  // Use the already calculated and smoothed value
                    target_angleControl_motors_[3] += -angle_increment_can11_motor1;



                    // CAN13 - Motor 1 (1-cos 적용)
                    float joy_input_can13_motor1 = msg->axes[axis_can13_motor1_];
                    if (invert_can13_motor1_) joy_input_can13_motor1 *= -1.0;
                    uint16_t maxSpeed_can13_motor1 = calculate_max_speed(joy_input_can13_motor1);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can13_motor1));
                    int32_t angle_increment_can13_motor1 = calculate_angle_increment_with_cos(joy_input_can13_motor1, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[4] += angle_increment_can13_motor1;



                    // CAN13 - Motor 2 (Combined Axes) (1-cos 적용)
                    float joy_input_can13_motor2_axis2 = -0.5 * (msg->axes[axis_can13_motor2_combined_[0]] - 1);
                    float joy_input_can13_motor2_axis5 = 0.5 * (msg->axes[axis_can13_motor2_combined_[1]] - 1);
                    float combined_joy_input_motor2 = joy_input_can13_motor2_axis2 + joy_input_can13_motor2_axis5;
                    if (invert_can13_motor2_) combined_joy_input_motor2 *= -1.0;
                    uint16_t maxSpeed_can13_motor2 = calculate_max_speed(combined_joy_input_motor2);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(combined_joy_input_motor2));
                    int32_t angle_increment_can13_motor2 = calculate_angle_increment_with_cos(combined_joy_input_motor2, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[5] += angle_increment_can13_motor2;

                    // CAN12 - Motor 1 (1-cos 적용)
                    float joy_input_can12_motor1 = msg->axes[axis_can12_motor1_];
                    if (invert_can12_motor1_) joy_input_can12_motor1 *= -1.0;
                    uint16_t maxSpeed_can12_motor1 = calculate_max_speed(joy_input_can12_motor1);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can12_motor1));
                    int32_t angle_increment_can12_motor1 = calculate_angle_increment_with_cos(joy_input_can12_motor1, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[6] += angle_increment_can12_motor1;



                    // CAN12 - Motor 2 (1-cos 적용)
                    float joy_input_can12_motor2 = msg->axes[axis_can12_motor2_];
                    if (invert_can12_motor2_) joy_input_can12_motor2 *= -1.0;
                    uint16_t maxSpeed_can12_motor2 = calculate_max_speed(joy_input_can12_motor2);
                    dynamic_angle_scale = std::max(min_angle_scale_, max_angle_scale_ * std::abs(joy_input_can12_motor2));
                    int32_t angle_increment_can12_motor2 = calculate_angle_increment_with_cos(joy_input_can12_motor2, dynamic_angle_scale, period_sec);
                    target_angleControl_motors_[7] += angle_increment_can12_motor2;


                     // Send commands to motors
                    process_motor_command(rmd, can_socket_10_, 1, target_angleControl_motors_[0], maxSpeed_can10_motor1, 0);
                    process_motor_command(rmd, can_socket_10_, 2, target_angleControl_motors_[1], maxSpeed_can10_motor2, 1);
                    process_motor_command(rmd, can_socket_11_, 1, target_angleControl_motors_[2], maxSpeed_can11_motor1, 2);
                    process_motor_command(rmd, can_socket_11_, 2, target_angleControl_motors_[3], maxSpeed_can11_motor1, 3);
                    process_motor_command(rmd, can_socket_13_, 1, target_angleControl_motors_[4], maxSpeed_can13_motor1, 4);
                    process_motor_command(rmd, can_socket_13_, 2, target_angleControl_motors_[5], maxSpeed_can13_motor2, 5);
                    process_motor_command(rmd, can_socket_12_, 1, target_angleControl_motors_[6], maxSpeed_can12_motor1, 6);
                    process_motor_command(rmd, can_socket_12_, 2, target_angleControl_motors_[7], maxSpeed_can12_motor2, 7);
                    
                }

                last_command_time_ = now;

             }else {
                 RCLCPP_WARN_ONCE(this->get_logger(), "No joystick messages received yet. Motor control is inactive.");
            }
        }
    }
    void process_motor_command(RMD_COMMAND& rmd, int socket_fd, int motor_id, int32_t target_angle, uint16_t max_speed, int motor_index) {
        last_angleControl_motors_[motor_index] = target_angle; // Directly use target angle

        RCLCPP_DEBUG(this->get_logger(), "CAN%d - Motor %d: Target Angle=%d,  MaxSpeed=%d",
                     (socket_fd == can_socket_10_ ? 10 : (socket_fd == can_socket_11_ ? 11 : (socket_fd == can_socket_13_ ? 13 : (socket_fd == can_socket_12_ ? 12 : socket_fd)))),
                     motor_id, target_angle,  max_speed);

        rmd.POSITION_CONTROL(socket_fd, motor_id, last_angleControl_motors_[motor_index], max_speed);
         RCLCPP_DEBUG(this->get_logger(), "Sent CAN Frame to CAN%d, Motor ID %d, Angle=%d, Speed=%d",
                        (socket_fd == can_socket_10_ ? 10 : (socket_fd == can_socket_11_ ? 11 : (socket_fd == can_socket_13_ ? 13 : (socket_fd == can_socket_12_ ? 12 : socket_fd)))),
                        motor_id, last_angleControl_motors_[motor_index], max_speed);
    }

    int can_socket_10_;
    int can_socket_11_;
    int can_socket_12_;
    int can_socket_13_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

bool set_realtime_scheduling() {
    struct sched_param sched_params;
    sched_params.sched_priority = 99;

    if (sched_setscheduler(0, SCHED_FIFO, &sched_params) != 0) {
        perror("sched_setscheduler failed");
        RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "Failed to set real-time scheduling (SCHED_FIFO).");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "Real-time scheduling (SCHED_FIFO) enabled.");

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        RCLCPP_FATAL(rclcpp::get_logger("rmd_control"), "Failed to lock memory (mlockall). This is critical for real-time performance. Exiting.");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "Memory locking (mlockall) successful.");
    return true;
}

int main(int argc, char *argv[]) {
    if (!set_realtime_scheduling()) {
        RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "Real-time setup failed. Exiting program.");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickToRMDControl>());
    rclcpp::shutdown();
    return 0;
}