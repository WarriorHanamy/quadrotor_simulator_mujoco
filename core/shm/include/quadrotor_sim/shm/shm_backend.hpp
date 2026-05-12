#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <stdexcept>

#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "quadrotor_sim/shm/shm_layout.hpp"

namespace quadrotor_sim::shm {

// ---------------------------------------------------------------------------
// State segment
// ---------------------------------------------------------------------------

inline StateWire* CreateStateShm() {
  FILE* f = fopen(kStateFile, "w");
  if (!f) throw std::runtime_error("shm_backend: cannot create state segment");
  if (ftruncate(fileno(f), sizeof(StateWire)) != 0)
    throw std::runtime_error("shm_backend: ftruncate state failed");
  fclose(f);

  f = fopen(kStateFile, "r+");
  if (!f) throw std::runtime_error("shm_backend: cannot reopen state segment");
  auto* ptr = reinterpret_cast<StateWire*>(
      mmap(nullptr, sizeof(StateWire), PROT_READ | PROT_WRITE, MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) throw std::runtime_error("shm_backend: mmap state failed");
  fclose(f);
  return ptr;
}

inline StateWire* OpenStateShm(bool write = false) {
  FILE* f = fopen(kStateFile, write ? "r+" : "r");
  if (!f) return nullptr;
  auto* ptr = reinterpret_cast<StateWire*>(
      mmap(nullptr, sizeof(StateWire),
           write ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) { fclose(f); return nullptr; }
  fclose(f);
  return ptr;
}

// ---------------------------------------------------------------------------
// Control segment
// ---------------------------------------------------------------------------

inline ControlWire* CreateCtrlShm() {
  FILE* f = fopen(kCtrlFile, "w");
  if (!f) throw std::runtime_error("shm_backend: cannot create ctrl segment");
  if (ftruncate(fileno(f), sizeof(ControlWire)) != 0)
    throw std::runtime_error("shm_backend: ftruncate ctrl failed");
  fclose(f);

  f = fopen(kCtrlFile, "r+");
  if (!f) throw std::runtime_error("shm_backend: cannot reopen ctrl segment");
  auto* ptr = reinterpret_cast<ControlWire*>(
      mmap(nullptr, sizeof(ControlWire), PROT_READ | PROT_WRITE, MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) throw std::runtime_error("shm_backend: mmap ctrl failed");
  fclose(f);
  return ptr;
}

inline ControlWire* OpenCtrlShm(bool write = false) {
  FILE* f = fopen(kCtrlFile, write ? "r+" : "r");
  if (!f) return nullptr;
  auto* ptr = reinterpret_cast<ControlWire*>(
      mmap(nullptr, sizeof(ControlWire),
           write ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) { fclose(f); return nullptr; }
  fclose(f);
  return ptr;
}

}  // namespace quadrotor_sim::shm
