#include "gimbal.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <utility>

namespace gimbal {

Gimbal::Gimbal(std::string device, uint32_t baudRate, unsigned periodMs)
    : device_(std::move(device)), baudRate_(baudRate), periodMs_(std::max(1u, periodMs)) {
    serial_.setPort(device_);
    serial_.setBaudrate(baudRate_);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.open();
    thread_ = std::thread(&Gimbal::run, this);
}

Gimbal::~Gimbal() { stop(); }

uint16_t Gimbal::toValue(double degrees, double maximumDegrees) {
    degrees = std::clamp(degrees, 0.0, maximumDegrees);
    return static_cast<uint16_t>(std::lround(degrees / maximumDegrees * 65535.0));
}

void Gimbal::publish(double yawDegrees, double pitchDegrees) {
    publish(Position{yawDegrees, pitchDegrees});
}

void Gimbal::publish(Position position) {
    position.yaw = std::clamp(position.yaw, 0.0, 180.0);     //yaw范围0-180°
    position.pitch = std::clamp(position.pitch, 0.0, 90.0);  //pitch范围0-90°
    std::lock_guard<std::mutex> lock(mutex_);
    target_ = position;
}

Position Gimbal::target() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_;
}

void Gimbal::sendFrame(Position position) {
    const uint16_t yaw = toValue(position.yaw, 180.0);
    const uint16_t pitch = toValue(position.pitch, 90.0);
    const std::array<uint8_t, 6> frame{
        Gimbal::kHeader, static_cast<uint8_t>(yaw >> 8), static_cast<uint8_t>(yaw),
        static_cast<uint8_t>(pitch >> 8), static_cast<uint8_t>(pitch), Gimbal::kTail};
    serial_.write(frame.data(), frame.size());
}

void Gimbal::run() {
    while (running_) {
        try {
            sendFrame(target());
        } catch (...) {
            running_ = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(periodMs_));
    }
}

void Gimbal::stop() {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
    if (serial_.isOpen()) serial_.close();
}

}  // namespace gimbal
