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

class RMD_COMMAND {
public:
    void TORQUE_CURRENT_CONTROL(int s, int ID, int16_t iqControl) {
        struct can_frame frame;

        // Torque current 값을 바이트로 분리
        uint8_t iqControl_low_byte = *reinterpret_cast<uint8_t *>(&iqControl);       // low byte
        uint8_t iqControl_high_byte = *(reinterpret_cast<uint8_t *>(&iqControl) + 1); // high byte

        // CAN 프레임 설정
        frame.can_id = 0x140 + ID; // CAN ID 설정
        frame.can_dlc = 8;        // 데이터 길이 설정
        frame.data[0] = 0xA1;     // 명령: Torque Current Control
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = iqControl_low_byte;
        frame.data[5] = iqControl_high_byte;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        // CAN 프레임을 소켓으로 전송
        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write failed");
        }
    }

    void SPEED_CONTROL(int s, int ID, int32_t speedControl) {
        struct can_frame frame;

        // Speed control 값을 바이트로 분리
        uint8_t speedControl_bytes[4];
        std::memcpy(speedControl_bytes, &speedControl, 4);

        // CAN 프레임 설정
        frame.can_id = 0x140 + ID; // CAN ID 설정
        frame.can_dlc = 8;        // 데이터 길이 설정
        frame.data[0] = 0xA2;     // 명령: Speed Control
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = speedControl_bytes[0];
        frame.data[5] = speedControl_bytes[1];
        frame.data[6] = speedControl_bytes[2];
        frame.data[7] = speedControl_bytes[3];

        // CAN 프레임을 소켓으로 전송
        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write failed");
        }
    }
};

class JoystickToRMDControl : public rclcpp::Node {
public:
    JoystickToRMDControl() : Node("joystick_to_rmd_control") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickToRMDControl::joy_callback, this, std::placeholders::_1));

        can_socket_10_ = open_can_socket("can10");
        can_socket_11_ = open_can_socket("can11");
        can_socket_12_ = open_can_socket("can12");
        can_socket_13_ = open_can_socket("can13");

        if (can_socket_10_ < 0 || can_socket_11_ < 0 || can_socket_12_ < 0 || can_socket_13_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open one or more CAN sockets. Exiting...");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN sockets opened successfully.");
        }
    }

    ~JoystickToRMDControl() {
        if (can_socket_10_ >= 0) {
            close(can_socket_10_);
        }
        if (can_socket_11_ >= 0) {
            close(can_socket_11_);
        }
        if (can_socket_12_ >= 0) {
            close(can_socket_12_);
        }
        if (can_socket_13_ >= 0) {
            close(can_socket_13_);
        }
    }

private:
    int open_can_socket(const char *interface_name) {
        int socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 소켓 생성
        if (socket_fd < 0) {
            perror("Socket creation failed");
            return -1;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name, IFNAMSIZ - 1);
        if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) { // 인터페이스 이름 가져오기
            perror("ioctl SIOCGIFINDEX failed");
            close(socket_fd);
            return -1;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { // 소켓 바인딩
            perror("Socket bind failed");
            close(socket_fd);
            return -1;
        }

        return socket_fd;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        RMD_COMMAND rmd;

        float torque_scale = 25.0; // Torque scaling 값
        int16_t iqControl0 = static_cast<int16_t>(msg->axes[3] * torque_scale);
        int16_t iqControl1 = static_cast<int16_t>(msg->axes[1] * torque_scale);
        int16_t iqControl4_0 = static_cast<int16_t>(msg->axes[4] * torque_scale);
        int16_t iqControl4_1 = static_cast<int16_t>(-msg->axes[4] * torque_scale);

        int32_t speed_scale = 1000000; // Speed scaling 값
        int32_t speedControl6 = static_cast<int32_t>(msg->axes[6] * speed_scale);
        int32_t speedControl7 = static_cast<int32_t>(msg->axes[7] * speed_scale);
      
        
        // 값 유효성 검사
        iqControl0 = std::clamp(iqControl0, static_cast<int16_t>(-32768), static_cast<int16_t>(32767));
        iqControl1 = std::clamp(iqControl1, static_cast<int16_t>(-32768), static_cast<int16_t>(32767));
        iqControl4_0 = std::clamp(iqControl4_0, static_cast<int16_t>(-32768), static_cast<int16_t>(32767));
        iqControl4_1 = std::clamp(iqControl4_1, static_cast<int16_t>(-32768), static_cast<int16_t>(32767));

        speedControl6 = std::clamp(speedControl6, -100000, 100000);
        speedControl7 = std::clamp(speedControl7, -100000, 100000);

        // 모터 ID 설정
        int motor_id1 = 1; // 모터 1 ID (CAN10, CAN11, CAN13)
        int motor_id2 = 2; // 모터 2 ID (CAN10, CAN11, CAN13)

        // Torque Current Control 명령 전송 (CAN10)
        rmd.TORQUE_CURRENT_CONTROL(can_socket_10_, motor_id1, iqControl0);
        rmd.TORQUE_CURRENT_CONTROL(can_socket_10_, motor_id2, iqControl1);

        // Torque Current Control 명령 전송 (CAN13)
        rmd.TORQUE_CURRENT_CONTROL(can_socket_11_, motor_id1, iqControl4_0);
        rmd.TORQUE_CURRENT_CONTROL(can_socket_11_, motor_id2, iqControl4_1);

        // Speed Control 명령 전송 (CAN11)
        rmd.SPEED_CONTROL(can_socket_13_, motor_id1, speedControl6);
        rmd.SPEED_CONTROL(can_socket_13_, motor_id2, speedControl7);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[3]: %f, Torque Current: %d", msg->axes[3], iqControl0);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[1]: %f, Torque Current: %d", msg->axes[1], iqControl1);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[4]: %f, Torque Current: %d", msg->axes[4], iqControl4_0);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[6]: %f, Speed Control: %d", msg->axes[6], speedControl6);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[7]: %f, Speed Control: %d", msg->axes[7], speedControl7);
    }

    int can_socket_10_;
    int can_socket_11_;
    int can_socket_12_;
    int can_socket_13_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickToRMDControl>());
    rclcpp::shutdown();
    return 0;
}