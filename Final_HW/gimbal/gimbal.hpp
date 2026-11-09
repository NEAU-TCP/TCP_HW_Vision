#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include "serial/serial.h"

namespace gimbal {

struct Position {
    double yaw = 0.0;    // 度, 0-180
    double pitch = 0.0;  // 度, 0-90
};

class Gimbal {
public:
    explicit Gimbal(
        std::string device = "/dev/ttyUSB0",  //串口号
        uint32_t baudRate = 115200,           //波特率
        unsigned periodMs = 10                //发送周期，ms
    );  
    ~Gimbal();

    Gimbal(const Gimbal&) = delete;
    Gimbal& operator=(const Gimbal&) = delete;

    void publish(double yawDegrees, double pitchDegrees);
    void publish(Position position);

    Position target() const;
    void stop();

private:
    void run();
    void sendFrame(Position position);
    static uint16_t toValue(double degrees, double maximumDegrees);

    inline static constexpr uint8_t kHeader = 0x0A;  //帧头
    inline static constexpr uint8_t kTail = 0xA5;   //帧尾

    std::string device_;
    uint32_t baudRate_;
    unsigned periodMs_;
    serial::Serial serial_;
    mutable std::mutex mutex_;
    Position target_{};
    std::atomic<bool> running_{true};
    std::thread thread_;
};

}  // namespace gimbal
