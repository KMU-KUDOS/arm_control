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
    void POSITION_CONTROL_COMMAND(int s, int ID, int32_t angleControl, uint16_t maxSpeed) {
        struct can_frame frame;

        // 각도를 바이트별로 나누기
        uint8_t angle_low_byte = (angleControl >> 0) & 0xFF;
        uint8_t angle_2nd_byte = (angleControl >> 8) & 0xFF;
        uint8_t angle_3rd_byte = (angleControl >> 16) & 0xFF;
        uint8_t angle_high_byte = (angleControl >> 24) & 0xFF;

        // 속도를 바이트별로 나누기
        uint8_t speed_low_byte = (maxSpeed >> 0) & 0xFF;
        uint8_t speed_high_byte = (maxSpeed >> 8) & 0xFF;

        // CAN 프레임 설정
        frame.can_id = 0x140 + ID; // CAN ID 설정
        frame.can_dlc = 8;        // 데이터 길이 설정
        frame.data[0] = 0xA4;     // 명령: Position Control Command
        frame.data[1] = 0x00;     // NULL
        frame.data[2] = speed_low_byte;
        frame.data[3] = speed_high_byte;
        frame.data[4] = angle_low_byte;
        frame.data[5] = angle_2nd_byte;
        frame.data[6] = angle_3rd_byte;
        frame.data[7] = angle_high_byte;

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

        can_socket_ = open_can_socket("can13");  // CAN 소켓 열기
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN socket. Exiting...");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN socket opened successfully.");
        }
    }

    ~JoystickToRMDControl() {
        if (can_socket_ >= 0) {
            close(can_socket_);
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

        // 축 [0]과 [1] 값을 기반으로 각도와 최대 속도 계산
        float angle_scale = 36000.0; // 1도 = 100단위
        float speed_scale = 100.0;  // 1dps = 100

        int32_t angleControl1 = static_cast<int32_t>(msg->axes[0] * angle_scale);
        uint16_t maxSpeed1 = static_cast<uint16_t>(std::abs(msg->axes[1] * speed_scale));

        int32_t angleControl2 = static_cast<int32_t>(msg->axes[2] * angle_scale);
        uint16_t maxSpeed2 = static_cast<uint16_t>(std::abs(msg->axes[3] * speed_scale));

        // Position Control 명령 전송 (모터 1)
        int motor_id1 = 1; // 모터 ID 1
        rmd.POSITION_CONTROL_COMMAND(can_socket_, motor_id1, angleControl1, maxSpeed1);

        // Position Control 명령 전송 (모터 2)
        int motor_id2 = 2; // 모터 ID 2
        rmd.POSITION_CONTROL_COMMAND(can_socket_, motor_id2, angleControl2, maxSpeed2);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Motor 1: Angle: %d, Max Speed: %d", angleControl1, maxSpeed1);
        RCLCPP_INFO(this->get_logger(), "Motor 2: Angle: %d, Max Speed: %d", angleControl2, maxSpeed2);
    }

    int can_socket_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickToRMDControl>());
    rclcpp::shutdown();
    return 0;
}

