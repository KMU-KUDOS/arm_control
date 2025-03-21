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
};

class JoystickToRMDControl : public rclcpp::Node {
public:
    JoystickToRMDControl() : Node("joystick_to_rmd_control") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoystickToRMDControl::joy_callback, this, std::placeholders::_1));

        can_socket_0_ = open_can_socket("can12");
        can_socket_1_ = open_can_socket("can13");
        

        if (can_socket_0_ < 0 || can_socket_1_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open one or more CAN sockets. Exiting...");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "CAN sockets opened successfully.");
        }
    }

    ~JoystickToRMDControl() {
        if (can_socket_0_ >= 0) {
            close(can_socket_0_);
        }
        if (can_socket_1_ >= 0) {
            close(can_socket_1_);
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

        
        float torque_scale = 25.0; // 스케일링 값 (조정 가능)
        int16_t iqControl0 = static_cast<int16_t>(msg->axes[3] * torque_scale);
        int16_t iqControl1 = static_cast<int16_t>(msg->axes[1] * torque_scale);
        int16_t iqControl4 = static_cast<int16_t>(msg->axes[4] * torque_scale);
        int16_t iqControl4_1 = static_cast<int16_t>(-msg->axes[4] * torque_scale);


        // 모터 ID 설정
        int motor_id1 = 1; // 모터 1 ID (CAN0)
        int motor_id2 = 2; // 모터 2 ID (CAN0)
        int motor_id3 = 1; // 모터 1 ID (CAN1)
        int motor_id4 = 2; // 모터 2 ID (CAN1)

        // Torque Current Control 명령 전송
        rmd.TORQUE_CURRENT_CONTROL(can_socket_0_, motor_id1, iqControl0);
        rmd.TORQUE_CURRENT_CONTROL(can_socket_0_, motor_id2, iqControl1);
        rmd.TORQUE_CURRENT_CONTROL(can_socket_1_, motor_id1, iqControl4);
        rmd.TORQUE_CURRENT_CONTROL(can_socket_1_, motor_id2, iqControl4_1);

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[3]: %f, Torque Current: %d", msg->axes[3], iqControl0);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[1]: %f, Torque Current: %d", msg->axes[1], iqControl1);
        RCLCPP_INFO(this->get_logger(), "Joystick Axes[4]: %f, Torque Current: %d", msg->axes[4], iqControl4);
       
    }

    int can_socket_0_;
    int can_socket_1_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickToRMDControl>());
    rclcpp::shutdown();
    return 0;
}
