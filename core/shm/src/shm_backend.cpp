#include "quadrotor_sim/shm/shm_layout.hpp"

#include <chrono>
#include <cstring>

namespace quadrotor_sim::shm {

// Seqlock helpers

void WriteBegin(uint64_t& seq) {
  seq++;
  __sync_synchronize();
}

void WriteEnd(uint64_t& seq) {
  __sync_synchronize();
  seq++;
}

// Wire ⟷ model conversion

void ToWire(const State& src, StateWire& dst) {
  dst.time = src.time;
  std::memcpy(dst.position, src.position, sizeof(src.position));
  std::memcpy(dst.orientation, src.orientation, sizeof(src.orientation));
  std::memcpy(dst.linear_velocity, src.linear_velocity, sizeof(src.linear_velocity));
  std::memcpy(dst.angular_velocity, src.angular_velocity, sizeof(src.angular_velocity));
  std::memcpy(dst.linear_acceleration, src.linear_acceleration, sizeof(src.linear_acceleration));
  dst.timestamp_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

void ToWire(const Control& src, ControlWire& dst) {
  dst.thrust = src.thrust;
  std::memcpy(dst.torque, src.torque, sizeof(src.torque));
  dst.timestamp_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
}

void FromWire(const StateWire& src, State& dst) {
  dst.time = src.time;
  std::memcpy(dst.position, src.position, sizeof(src.position));
  std::memcpy(dst.orientation, src.orientation, sizeof(src.orientation));
  std::memcpy(dst.linear_velocity, src.linear_velocity, sizeof(src.linear_velocity));
  std::memcpy(dst.angular_velocity, src.angular_velocity, sizeof(src.angular_velocity));
  std::memcpy(dst.linear_acceleration, src.linear_acceleration, sizeof(src.linear_acceleration));
}

void FromWire(const ControlWire& src, Control& dst) {
  dst.thrust = src.thrust;
  std::memcpy(dst.torque, src.torque, sizeof(src.torque));
}

}  // namespace quadrotor_sim::shm
