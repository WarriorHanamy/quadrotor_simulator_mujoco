#pragma once

/**
 * ABI-stable wire format for shared-memory IPC.
 *
 * Defines packed structs, seqlock protocol, file paths, and
 * conversion functions between wire format and core domain types.
 */

#include <cstdint>

#include "quadrotor_sim/types.hpp"

namespace quadrotor_sim::shm {

#pragma pack(push, 1)

struct StateWire {
  uint64_t sequence;
  double   time;
  double   position[3];
  double   orientation[4];
  double   linear_velocity[3];
  double   angular_velocity[3];
  double   linear_acceleration[3];
  uint64_t timestamp_ns;
  uint8_t  _pad[40];
};
static_assert(sizeof(StateWire) == 192, "StateWire must be 192 bytes");

struct ControlWire {
  uint64_t sequence;
  double   thrust;
  double   torque[3];
  uint64_t timestamp_ns;
  uint8_t  _pad[16];
};
static_assert(sizeof(ControlWire) == 64, "ControlWire must be 64 bytes");

#pragma pack(pop)

// File paths
inline constexpr const char* kBaseDir   = "/dev/shm/quadrotor_sim";
inline constexpr const char* kStateFile = "/dev/shm/quadrotor_sim/state";
inline constexpr const char* kCtrlFile  = "/dev/shm/quadrotor_sim/ctrl";

// Wire ⟷ model conversion
void ToWire(const State& src, StateWire& dst);
void ToWire(const Control& src, ControlWire& dst);
void FromWire(const StateWire& src, State& dst);
void FromWire(const ControlWire& src, Control& dst);

// Seqlock protocol
void WriteBegin(uint64_t& seq);
void WriteEnd(uint64_t& seq);

template<typename T>
bool ReadConsistent(const volatile T& src, T& dst) {
  uint64_t before = src.sequence;
  if (before & 1ULL) return false;
  __sync_synchronize();
  dst = const_cast<const T&>(src);
  __sync_synchronize();
  uint64_t after = src.sequence;
  return before == after;
}

template<typename T>
bool ReadConsistent(const T& src, T& dst) {
  return ReadConsistent(*reinterpret_cast<const volatile T*>(&src), dst);
}

}  // namespace quadrotor_sim::shm
