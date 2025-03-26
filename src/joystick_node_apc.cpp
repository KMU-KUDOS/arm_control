#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
#include <termios.h>
#include <fcntl.h>

using namespace std::chrono_literals;

// CAN ID 상수 정의
const uint32_t RMD_X8_PRO_BASE_ID = 0x140; // can10, can11
const uint32_t RMD_L9025_BASE_ID = 0x140;    // can12, can13 (가정)

// 각 모터 타입에 대한 TICKS_PER_DEGREE 정의
const double TICKS_PER_DEGREE_X8_PRO = 1296000.0 / 360.0;  // can10, can11
const double TICKS_PER_DEGREE_L9025 = 288000.0 / 360.0;   // can12, can13

// 주기를 ns 단위로 정의 (10ms)
const long long ns_TenMilSec = 10000000;

// 모터와 CANable 개수를 정의
const int NUM_OF_MOTOR = 2;
const int NUM_OF_CANABLE = 4;

// 키보드 입력 확인 함수
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

// 주기 정보 구조체
struct period_info {
    struct timespec next_period;
    struct timespec current_time_1;
    long long period_ns;
};

// RMD 모터 커맨드 클래스
class RMD_COMMAND {
public:
    // 위치 제어 커맨드
    void POSITION_CONTROL(int s, int ID, double angleControl, uint16_t maxSpeed, bool is_L9025, bool reverse_direction = false) {
        struct can_frame frame;
        uint8_t maxSpeed_bytes[2];
        uint8_t angleControl_bytes[4];

        // 모터 타입에 따라 각도를 ticks로 변환
        int32_t angle_ticks;
        if (is_L9025) {
            angle_ticks = static_cast<int32_t>(angleControl * TICKS_PER_DEGREE_L9025);
        } else {
            angle_ticks = static_cast<int32_t>(angleControl * TICKS_PER_DEGREE_X8_PRO);
        }

        // 회전 방향 반전 처리
        if (reverse_direction) {
            angle_ticks = -angle_ticks;
        }

        // Little-endian (angle_ticks)
        angleControl_bytes[0] = angle_ticks & 0xFF;
        angleControl_bytes[1] = (angle_ticks >> 8) & 0xFF;
        angleControl_bytes[2] = (angle_ticks >> 16) & 0xFF;
        angleControl_bytes[3] = (angle_ticks >> 24) & 0xFF;

        // Big-endian (maxSpeed)
        maxSpeed_bytes[0] = (maxSpeed >> 8) & 0xFF;
        maxSpeed_bytes[1] = maxSpeed & 0xFF;

        // CAN ID 설정 (모터 타입과 ID에 따라)
        frame.can_id = (is_L9025 ? RMD_L9025_BASE_ID : RMD_X8_PRO_BASE_ID) + ID;
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

        // 전송된 CAN 프레임 로깅
        std::stringstream ss;
        ss << "Sent CAN Frame (Hex) - ID: 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id
            << ", Data: 0x";
        for (int i = 0; i < frame.can_dlc; ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("RMD_COMMAND"), "%s", ss.str().c_str());
    }

    // Raw CAN 프레임 전송 함수
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
            // 로깅 조건 (필요에 따라 수정)
            bool should_log = false;
            if (data.size() >= 2 && data[0] == 0xA4 && data[1] != 0x00) {
                should_log = true;
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

    // 모터 데이터 구조체 (온도, 토크 전류, 속도, 엔코더, 각도 포함)
    struct MotorData {
        int16_t temperature;
        int16_t iq;
        int16_t speed;
        int16_t encoder;
        double angle;
    };

    // 토크, 속도, 각도, 온도, 엔코더 읽기 함수
   MotorData READ_TORQUE_SPEED_ANGLE(int s, int can_id_offset, int motor_id, struct can_frame& frame) {
        MotorData data;
        // can_id_offset 사용 X, motor_id만 사용
        frame.can_id = RMD_X8_PRO_BASE_ID + motor_id;  // 또는 RMD_L9025_BASE_ID + motor_id (L9025인 경우)

        frame.can_dlc = 8;
        frame.data[0] = 0x9C; // 다회전 각도 읽기 커맨드
        memset(&frame.data[1], 0, 7);

        if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Write failed");
            return data;
        }

        struct can_frame rx_frame;
        if (read(s, &rx_frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("Read failed");
            return data;
        }

        // 올바른 CAN ID를 수신했는지 확인 (motor_id만 사용)
        if (rx_frame.can_id == (RMD_X8_PRO_BASE_ID + motor_id) || rx_frame.can_id == (RMD_L9025_BASE_ID + motor_id)) {
            if (rx_frame.data[0] == 0x9C) { // 0x9C 커맨드인지 확인
                 // 데이터 파싱 (Little Endian)
                data.temperature = rx_frame.data[1];
                data.iq = (static_cast<int16_t>(rx_frame.data[3]) << 8) | rx_frame.data[2];
                data.speed = (static_cast<int16_t>(rx_frame.data[5]) << 8) | rx_frame.data[4];
                data.encoder = (static_cast<int16_t>(rx_frame.data[7]) << 8) | rx_frame.data[6];


                // 각도 계산 (Multi-turn)
                int64_t raw_angle = 0;
                for (int i = 7; i >= 1; --i)
                {
                    raw_angle = (raw_angle << 8) | rx_frame.data[i];
                }

                if (raw_angle >= 0x80000000000000)
                {
                    raw_angle -= 0x100000000000000;
                }

                // 모터 타입에 따라 각도 계산
                if (rx_frame.can_id == (RMD_X8_PRO_BASE_ID + motor_id)) // motor_id만 사용
                {
                    data.angle = static_cast<double>(raw_angle) / 1296000.0 * 360.0;
                }
                else if (rx_frame.can_id == (RMD_L9025_BASE_ID + motor_id)) // motor_id만 사용
                {
                    data.angle = static_cast<double>(raw_angle) / 288000.0 * 360.0;
                }
            }
        }


        return data;
    }
};

// ROS2 노드 클래스
class InteractiveAngleControl : public rclcpp::Node {
public:
    InteractiveAngleControl() : Node("interactive_angle_control"), current_angles_(8, 0.0), reversed_motors_(8, false) {
        // 파라미터 선언
        this->declare_parameter<std::string>("can_interface_10", "can10");
        this->declare_parameter<std::string>("can_interface_11", "can11");
        this->declare_parameter<std::string>("can_interface_12", "can12");
        this->declare_parameter<std::string>("can_interface_13", "can13");
        can_interface_10_ = this->get_parameter("can_interface_10").as_string();
        can_interface_11_ = this->get_parameter("can_interface_11").as_string();
        can_interface_12_ = this->get_parameter("can_interface_12").as_string();
        can_interface_13_ = this->get_parameter("can_interface_13").as_string();

        this->declare_parameter<int>("max_speed", 1);
        max_speed_scale_ = this->get_parameter("max_speed").as_int();

        // CAN 소켓 열기
        can_socket_10_ = open_can_socket(can_interface_10_.c_str());
        can_socket_11_ = open_can_socket(can_interface_11_.c_str());
        can_socket_12_ = open_can_socket(can_interface_12_.c_str());
        can_socket_13_ = open_can_socket(can_interface_13_.c_str());

        if (can_sockets_ok()) {
            RCLCPP_INFO(this->get_logger(), "CAN sockets opened successfully.");
        } else {
            RCLCPP_FATAL(this->get_logger(), "Failed to open one or more CAN sockets. Exiting...");
            rclcpp::shutdown();
            return;
        }

        last_angleControl_motors_.resize(8, 0.0);

        // can11에 연결된 모터 (motor_index 2, 3)는 회전 방향 반전
        reversed_motors_[2] = true;
        reversed_motors_[3] = true;

        RMD_COMMAND rmd;
        send_startup_raw_frames(rmd); // 초기 위치 설정
        RCLCPP_INFO(this->get_logger(), "Raw CAN frames sent at startup for initial position.");

        // Publisher 생성
        motor_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("motor_angles", 10);

        // 스레드 생성 및 시작
        main_thread_exit = false;
        pthread_create(&main_thread, NULL, &InteractiveAngleControl::pthread_main_static, this);

        input_thread_ = std::thread(&InteractiveAngleControl::input_loop, this);
    }

    // 정적 멤버 함수 (스레드 함수 래퍼)
    static void* pthread_main_static(void* arg) {
        InteractiveAngleControl* instance = static_cast<InteractiveAngleControl*>(arg);
        return instance->pthread_main();
    }

    ~InteractiveAngleControl() {
        main_thread_exit = true;
        pthread_join(main_thread, NULL);
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        close_can_sockets();
    }

private:
    // CAN 소켓 상태 확인
    bool can_sockets_ok() const {
        return can_socket_10_ >= 0 && can_socket_11_ >= 0 && can_socket_12_ >= 0 && can_socket_13_ >= 0;
    }

    // CAN 소켓 닫기
    void close_can_sockets() {
        if (can_socket_10_ >= 0) close(can_socket_10_);
        if (can_socket_11_ >= 0) close(can_socket_11_);
        if (can_socket_12_ >= 0) close(can_socket_12_);
        if (can_socket_13_ >= 0) close(can_socket_13_);
    }

    // CAN 소켓 열기
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

    // 시작 시 Raw CAN 프레임 전송 (초기 위치 설정)
    void send_startup_raw_frames(RMD_COMMAND& rmd) {
        uint32_t raw_can_id1 = 0x141;
        uint32_t raw_can_id2 = 0x142;
        std::vector<uint8_t> raw_can_data_initial_pos = {0xA4, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00};

        if (can_sockets_ok()) {
            // can10, can11 (RMD-X8 Pro) 초기화

            rmd.SEND_RAW_FRAME(can_socket_10_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_10_, raw_can_id2, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_11_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_11_, raw_can_id2, raw_can_data_initial_pos);

            // can12, can13 (RMD-L9025) 초기화
            rmd.SEND_RAW_FRAME(can_socket_12_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_12_, raw_can_id2, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_13_, raw_can_id1, raw_can_data_initial_pos);
            rmd.SEND_RAW_FRAME(can_socket_13_, raw_can_id2, raw_can_data_initial_pos);

            std::this_thread::sleep_for(10s); // 5초 대기
        }
    }
// 메인 스레드 함수 (10ms 주기로 0x9C 커맨드 전송 및 토픽 발행)
    void* pthread_main() {
        struct period_info pinfo;
        pinfo.period_ns = ns_TenMilSec; // 10ms 주기로 설정
        clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period)); // 초기 시간 설정

        RMD_COMMAND rmd;
        std::vector<struct can_frame> frame(NUM_OF_CANABLE);  // 크기 지정
        std::vector<RMD_COMMAND::MotorData> motor_data(NUM_OF_CANABLE * NUM_OF_MOTOR); // 크기 지정: canable * 2

        while (!main_thread_exit) {
            pthread_testcancel(); // 스레드 취소 지점 확인

            // 현재 시간 가져오기 (주기 계산을 위해)
            clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1);

            // 모든 CAN 인터페이스와 모터에 대해 0x9C 커맨드 전송 및 데이터 읽기
            int motor_index = 0;
              for (int i = 0; i < NUM_OF_CANABLE; i++) { // can_id_offset 대신 사용
                int can_socket;
                if (i == 0) can_socket = can_socket_10_;
                else if (i == 1) can_socket = can_socket_11_;
                else if (i == 2) can_socket = can_socket_12_;
                else can_socket = can_socket_13_;  // i == 3

                for (int j = 1; j <= NUM_OF_MOTOR; j++) { // motor_id는 1 또는 2
                    motor_data[motor_index] = rmd.READ_TORQUE_SPEED_ANGLE(can_socket, i, j, frame[i]); // can_socket과 j 전달
                    motor_index++;
                }
            }


            // 읽은 각도를 Float64MultiArray 메시지로 퍼블리시
            std_msgs::msg::Float64MultiArray angle_msg;
            for (const auto& data : motor_data) {
                angle_msg.data.push_back(data.angle);
            }
            motor_angle_publisher_->publish(angle_msg);

            // 다음 주기까지 대기 (정확한 10ms 주기를 위해)
            pinfo.next_period.tv_nsec += pinfo.period_ns;
            while (pinfo.next_period.tv_nsec >= 1000000000) {
                pinfo.next_period.tv_nsec -= 1000000000;
                pinfo.next_period.tv_sec++;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL); // 정해진 시간까지 대기

        }
        return nullptr;
    }

    // 사용자 입력 처리 스레드 함수 (별도 스레드, 필요에 따라 주석 처리 가능)
    void input_loop() {
        RMD_COMMAND rmd;
        int motor_index;
        double angle;
        char command;
        bool input_mode = false;

        while (rclcpp::ok()) {
            if (!input_mode) {
                std::cout << "Enter command (m: set motor, q: quit): ";
                std::cout.flush();
                input_mode = true;
            }

            if (kbhit()) {
                std::cin >> command;

                if (command == 'q') {
                    break;
                } else if (command == 'm') {
                    std::cout << "Enter motor index (0-7): ";
                    std::cout.flush();
                    if (std::cin >> motor_index) {
                        if (motor_index >= 0 && motor_index < 8) {
                            std::cout << "Enter angle (degrees): ";
                            std::cout.flush();
                            if (std::cin >> angle) {
                                current_angles_[motor_index] = angle;

                                // 모터 인덱스에 따라 적절한 process_motor_command 호출
                                if (motor_index == 0) process_motor_command(rmd, can_socket_10_, 1, current_angles_[0], max_speed_scale_, 0, false);
                                else if (motor_index == 1) process_motor_command(rmd, can_socket_10_, 2, current_angles_[1], max_speed_scale_, 1, false);
                                else if (motor_index == 2) {
                                   
                                    process_motor_command(rmd, can_socket_11_, 1, current_angles_[2], max_speed_scale_, 2, false);
                                    process_motor_command(rmd, can_socket_11_, 2, -1 * current_angles_[2], max_speed_scale_, 2, false);
                                }
                                else if (motor_index == 4) process_motor_command(rmd, can_socket_12_, 1, current_angles_[4], max_speed_scale_, 4, true); 
                                else if (motor_index == 5) process_motor_command(rmd, can_socket_12_, 2, current_angles_[5], max_speed_scale_, 5, true);
                                else if (motor_index == 6) process_motor_command(rmd, can_socket_13_, 1, current_angles_[6], max_speed_scale_, 6, true);
                                else if (motor_index == 7) process_motor_command(rmd, can_socket_13_, 2, current_angles_[7], max_speed_scale_, 7, true);
                            } else {
                                RCLCPP_WARN(this->get_logger(), "Invalid angle input. Please enter a numeric value.");
                                std::cin.clear();
                                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Invalid motor index: %d", motor_index);
                            std::cin.clear();
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Invalid input. Please enter motor index.");
                        std::cin.clear();
                        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    }
                    input_mode = false;
                } else {
                    std::cout << "Invalid command." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    input_mode = false;
                }
            }
             std::this_thread::sleep_for(10ms);
        }

    }

     // 모터 커맨드 처리 함수 (여전히 필요함 - 사용자 입력 처리)
    void process_motor_command(RMD_COMMAND& rmd, int socket_fd, int motor_id, double target_angle, uint16_t max_speed, int motor_index, bool is_L9025) {
        bool reverse = reversed_motors_[motor_index];
        last_angleControl_motors_[motor_index] = target_angle;


          RCLCPP_DEBUG(this->get_logger(), "CAN%d - Motor %d: Target Angle=%.2f,  MaxSpeed=%d, is_L9025=%s, reverse=%s",
                     (socket_fd == can_socket_10_ ? 10 : (socket_fd == can_socket_11_ ? 11 : (socket_fd == can_socket_13_ ? 13 : (socket_fd == can_socket_12_ ? 12 : socket_fd)))),
                     motor_id, target_angle, max_speed, (is_L9025 ? "true" : "false"), (reverse ? "true" : "false"));


        rmd.POSITION_CONTROL(socket_fd, motor_id, last_angleControl_motors_[motor_index], max_speed, is_L9025, reverse);

         RCLCPP_DEBUG(this->get_logger(), "Sent CAN Frame to CAN%d, Motor ID %d, Angle=%.2f, Speed=%d, is_L9025=%s, reverse=%s",
                    (socket_fd == can_socket_10_ ? 10 : (socket_fd == can_socket_11_ ? 11 : (socket_fd == can_socket_13_ ? 13 : (socket_fd == can_socket_12_ ? 12 : socket_fd)))),
                    motor_id, last_angleControl_motors_[motor_index], max_speed, (is_L9025 ? "true" : "false"), (reverse ? "true" : "false"));
    }

    // 멤버 변수
    int can_socket_10_;
    int can_socket_11_;
    int can_socket_12_;
    int can_socket_13_;
    std::vector<double> last_angleControl_motors_;
    uint16_t max_speed_scale_;
    std::string can_interface_10_;
    std::string can_interface_11_;
    std::string can_interface_12_;
    std::string can_interface_13_;
    std::vector<double> current_angles_;
    std::thread input_thread_;
    std::vector<bool> reversed_motors_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_angle_publisher_;
    bool main_thread_exit;
    pthread_t main_thread;
};

// 실시간 스케줄링 설정 함수
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
    auto node = std::make_shared<InteractiveAngleControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}