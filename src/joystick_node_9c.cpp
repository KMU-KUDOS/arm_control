#include <iostream>
#include <string>
#include <cstdio>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <array>
#include <unistd.h> // usleep, fd_set, select
#include <errno.h> // errno

#include <sched.h>
#include <sys/mman.h>
#include <rclcpp/rclcpp.hpp> // ROS 2 로깅 (필요한 경우, 로깅 기능 제거 가능)

using namespace std;

// 모터 데이터를 저장할 구조체 (기어비 정보 추가)
struct MotorData {
    int temp_value = 0;
    int torque_value = 0;
    int speed_value = 0;
    int current_encoder_value = 0;
    double total_degree = 0.0;
    double total_radian = 0.0;
    int rotation_count = 0;
    double gear_ratio = 36.0; // 기어비 추가
};

// 실시간 스케줄링 설정 함수
bool set_realtime_scheduling() {
    struct sched_param sched_params;
    sched_params.sched_priority = 98;

    if (sched_setscheduler(0, SCHED_FIFO, &sched_params) != 0) {
        perror("sched_setscheduler failed");
        RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "실시간 스케줄링 (SCHED_FIFO) 설정 실패.");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "실시간 스케줄링 (SCHED_FIFO) 활성화.");

    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        RCLCPP_FATAL(rclcpp::get_logger("rmd_control"), "메모리 잠금 (mlockall) 실패. 실시간 성능에 매우 중요합니다. 프로그램 종료.");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "메모리 잠금 (mlockall) 성공.");

    return true;
}

// 각도 계산 함수 (MotorData 구조체와 기어비 사용)
double calculate_angle(int current_encoder_value, int previous_encoder_value, int& rotation_count, double gear_ratio) {
    int delta_encoder = current_encoder_value - previous_encoder_value;

    // 오버플로우/언더플로우 처리
    if (delta_encoder > 32768) {
        delta_encoder -= 65536;
        rotation_count--;
    } else if (delta_encoder < -32768) {
        delta_encoder += 65536;
        rotation_count++;
    }

    // 총 엔코더 값 계산 (회전 수 고려)
    double total_encoder_value = (double)current_encoder_value + (double)rotation_count * 65536.0;

    // 각도 계산 (기어비 반영)
    double degree = (total_encoder_value / 65536.0) * 360.0 / gear_ratio;

    // 각도를 0~360 범위로 조정
    degree = fmod(degree, 360.0); // fmod 사용
    if (degree < 0) {
        degree += 360.0;
    }

    return degree;
}

int main(int argc, char *argv[]) {
    if (!set_realtime_scheduling()) {
        RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "실시간 설정 실패. 프로그램 종료.");
        return 1;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "실시간 설정 성공, 데이터 읽기 진행.");
    }

    rclcpp::init(argc, argv);

    // CAN 인터페이스 목록
    vector<string> interfaces = {"can10", "can11", "can12", "can13"};
    map<string, FILE*> pipes;
    map<string, map<int, MotorData>> motor_data_map; // MotorData 구조체 사용
    map<string, map<int, int>> previous_encoder_values; // 이전 엔코더 값 저장
  //  map<string, map<int, int>> rotation_counts; -> calculate_angle 함수 내에서 지역변수로 처리

    // 각 CAN 인터페이스에 대해 candump 파이프 열기
    for (const string& interface : interfaces) {
        string command = "candump " + interface;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            cerr << interface << " 파이프 열기 오류!" << endl;
            RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "%s 파이프 열기 실패.", interface.c_str());

            for(auto& p:pipes) {
                pclose(p.second);
            }
            rclcpp::shutdown();
            return 1;
        }
        pipes[interface] = pipe;
        RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "%s 파이프 열기 성공.", interface.c_str());
    }

    char buffer[128];
    fd_set read_fds;
    int max_fd = 0;

    // 파일 디스크립터 최댓값 갱신 (select() 함수를 위해 필요)
    for (const auto& pair : pipes) {
        if (fileno(pair.second) > max_fd) {
            max_fd = fileno(pair.second);
        }
    }

    while (rclcpp::ok()) { // ROS 2 종료 신호 처리
        FD_ZERO(&read_fds); // fd_set 초기화
        for (const auto& pair : pipes) {
            FD_SET(fileno(pair.second), &read_fds); // fd_set에 파일 디스크립터 추가
        }

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10ms 타임아웃 (필요에 따라 조정)

        int select_result = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout); // select() 호출

        if (select_result == -1) {
            perror("select error");
            RCLCPP_ERROR(rclcpp::get_logger("rmd_control"), "select() 오류 발생: %s", strerror(errno));
            break;
        } else if (select_result > 0) { // 읽을 데이터가 있는 파이프가 있는 경우

            for (const auto& pair : pipes) {
                string interface_name = pair.first;
                FILE* pipe = pair.second;

                if (FD_ISSET(fileno(pipe), &read_fds)) {
                    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                        string line(buffer);
                        stringstream ss(line);
                        string candump_interface;
                        string temp_canId_str;
                        string bracket_length;
                        string data_str;

                        ss >> candump_interface >> temp_canId_str >> bracket_length;
                        getline(ss, data_str);

                        int canId = 0;
                        try{
                            canId = stoi(temp_canId_str, nullptr, 16);
                        }
                        catch(const std::invalid_argument & e){
                            RCLCPP_WARN(rclcpp::get_logger("rmd_control"), "%s, CAN ID 파싱 오류 (stoi): %s, 문자열: %s", interface_name.c_str(), e.what(), temp_canId_str.c_str());
                            continue;

                        }
                        catch (const std::out_of_range& e) {
                                RCLCPP_WARN(rclcpp::get_logger("rmd_control"), "%s, CAN ID 파싱 오류 (out_of_range): %s, 문자열: %s", interface_name.c_str(), e.what(), temp_canId_str.c_str());
                                continue;
                        }

                        data_str.erase(0, data_str.find_first_not_of(" "));

                        stringstream data_ss(data_str);
                        vector<int> data_bytes;
                        string byte_str;

                        while(data_ss >> byte_str){
                            try{
                                data_bytes.push_back(stoi(byte_str, nullptr, 16));
                            }
                            catch(const std::invalid_argument& e){
                                 RCLCPP_WARN(rclcpp::get_logger("rmd_control"), "%s, CAN ID %d 데이터 파싱 오류 (stoi): %s, 바이트 문자열: %s", interface_name.c_str(), canId, e.what(), byte_str.c_str());
                                data_bytes.clear(); // 파싱 오류 발생 시 data_bytes 초기화하고 다음 라인 처리
                                break; // 바이트 파싱 오류 발생 시 현재 라인 처리 중단

                            }
                            catch (const std::out_of_range& e) {
                                RCLCPP_WARN(rclcpp::get_logger("rmd_control"), "%s, CAN ID %d 데이터 파싱 오류 (out_of_range): %s, 바이트 문자열: %s", interface_name.c_str(), canId, e.what(), byte_str.c_str());
                                data_bytes.clear(); // 파싱 오류 발생 시 data_bytes 초기화하고 다음 라인 처리
                                break; // 바이트 파싱 오류 발생 시 현재 라인 처리 중단
                            }
                        }
                        if(data_bytes.empty()) continue;

                        if ((canId == 0x141 || canId == 0x142) && data_bytes.size() >= 8 && (data_bytes[1] != 0x00)) { // 원래 조건
                            RCLCPP_DEBUG(rclcpp::get_logger("rmd_control"), "%s, CAN ID %d, 조건 통과, data_bytes.size(): %zu", interface_name.c_str(), canId, data_bytes.size()); // 조건 통과 로그

                            int temp_byte_low = data_bytes[1];
                            int torque_byte_low = data_bytes[2];
                            int torque_byte_high = data_bytes[3];
                            int speed_byte_low = data_bytes[4];
                            int speed_byte_high = data_bytes[5];
                            int encoder_byte_low = data_bytes[6];
                            int encoder_byte_high = data_bytes[7];

                            int current_encoder_value = (encoder_byte_high << 8) + encoder_byte_low;
                            int torque_value = (torque_byte_high << 8) + torque_byte_low;
                            int speed_value = (speed_byte_high << 8) + speed_byte_low;
                            int temp_value = temp_byte_low;

                            MotorData& motor_data = motor_data_map[interface_name][canId];
                            int rotation_count = motor_data.rotation_count; //회전수 변수 가져옴

                            // 이전 엔코더 값이 없으면 초기화
                            if (previous_encoder_values[interface_name].find(canId) == previous_encoder_values[interface_name].end()) {
                                previous_encoder_values[interface_name][canId] = current_encoder_value;
                                RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "%s, CAN ID %d 초기 엔코더 값: %d", interface_name.c_str(), canId, current_encoder_value);
                            }

                            // 각도 계산 (함수 호출)
                            double degree = calculate_angle(current_encoder_value, previous_encoder_values[interface_name][canId], rotation_count, motor_data.gear_ratio);
                            double radian = degree * M_PI / 180.0;

                            // MotorData 구조체에 데이터 저장
                            motor_data.temp_value = temp_value;
                            motor_data.torque_value = torque_value;
                            motor_data.speed_value = speed_value;
                            motor_data.current_encoder_value = current_encoder_value;
                            motor_data.total_degree = degree;
                            motor_data.total_radian = radian;
                            motor_data.rotation_count = rotation_count; // 업데이트된 rotation_count 저장


                            RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "%s, CAN ID: %d, Temp: %d C, Torque: %d, Speed: %d, Raw Encoder: %d, Rotations: %d, Total Degree: %.2f deg",
                                        interface_name.c_str(), canId, temp_value, torque_value, speed_value, current_encoder_value, motor_data.rotation_count, degree);


                            previous_encoder_values[interface_name][canId] = current_encoder_value; // 이전 엔코더 값 업데이트

                         } else {
                            RCLCPP_DEBUG(rclcpp::get_logger("rmd_control"), "%s, CAN ID %d, 조건 불만족 (CAN ID: %d, data_bytes.size(): %zu)", interface_name.c_str(), canId, canId, data_bytes.size()); // 조건 불만족 로그
                        }
                    } else {
                         RCLCPP_WARN(rclcpp::get_logger("rmd_control"), "%s에서 fgets 오류 발생", interface_name.c_str()); // fgets 오류 로그
                        if (ferror(pipe)) { // 더 자세한 오류 정보 출력
                            perror("fgets error");
                        }
                        clearerr(pipe); // 오류 플래그 초기화
                    }
                }
            }
        }

        usleep(100000); // 10ms 주기 (select timeout과 동일하게 유지하거나 필요에 따라 조정)
    }

      // 파이프 닫기
    for (const auto& pair : pipes) {
        pclose(pair.second);
        RCLCPP_INFO(rclcpp::get_logger("rmd_control"), "%s 파이프 닫기 성공.", pair.first.c_str());
    }

    rclcpp::shutdown();

    return 0;
}