#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <stdexcept>

#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sim_schema.h"

/**
 * Create or attach to shared memory segments under /dev/shm/quadrotor_sim/.
 *
 * The core (owner) calls create_*_shm(); adapters call open_*_shm().
 * All functions return a typed pointer to the memory-mapped struct.
 */

// ---------------------------------------------------------------------------
// State segment
// ---------------------------------------------------------------------------

inline QuadrotorState* create_state_shm() {
  FILE* f = fopen(SHM_STATE_FILE, "w");
  if (!f) throw std::runtime_error("shm_backend: cannot create state segment");
  if (ftruncate(fileno(f), sizeof(QuadrotorState)) != 0)
    throw std::runtime_error("shm_backend: ftruncate state failed");
  fclose(f);

  f = fopen(SHM_STATE_FILE, "r+");
  if (!f) throw std::runtime_error("shm_backend: cannot reopen state segment");
  auto* ptr = reinterpret_cast<QuadrotorState*>(
      mmap(nullptr, sizeof(QuadrotorState), PROT_READ | PROT_WRITE,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) throw std::runtime_error("shm_backend: mmap state failed");
  fclose(f);
  return ptr;
}

inline QuadrotorState* open_state_shm(bool write = false) {
  FILE* f = fopen(SHM_STATE_FILE, write ? "r+" : "r");
  if (!f) return nullptr;
  auto* ptr = reinterpret_cast<QuadrotorState*>(
      mmap(nullptr, sizeof(QuadrotorState),
           write ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) { fclose(f); return nullptr; }
  fclose(f);
  return ptr;
}

// ---------------------------------------------------------------------------
// Control segment
// ---------------------------------------------------------------------------

inline QuadrotorControl* create_ctrl_shm() {
  FILE* f = fopen(SHM_CTRL_FILE, "w");
  if (!f) throw std::runtime_error("shm_backend: cannot create ctrl segment");
  if (ftruncate(fileno(f), sizeof(QuadrotorControl)) != 0)
    throw std::runtime_error("shm_backend: ftruncate ctrl failed");
  fclose(f);

  f = fopen(SHM_CTRL_FILE, "r+");
  if (!f) throw std::runtime_error("shm_backend: cannot reopen ctrl segment");
  auto* ptr = reinterpret_cast<QuadrotorControl*>(
      mmap(nullptr, sizeof(QuadrotorControl), PROT_READ | PROT_WRITE,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) throw std::runtime_error("shm_backend: mmap ctrl failed");
  fclose(f);
  return ptr;
}

inline QuadrotorControl* open_ctrl_shm(bool write = false) {
  FILE* f = fopen(SHM_CTRL_FILE, write ? "r+" : "r");
  if (!f) return nullptr;
  auto* ptr = reinterpret_cast<QuadrotorControl*>(
      mmap(nullptr, sizeof(QuadrotorControl),
           write ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) { fclose(f); return nullptr; }
  fclose(f);
  return ptr;
}

// ---------------------------------------------------------------------------
// Image segment
// ---------------------------------------------------------------------------

inline ImageData* create_image_shm() {
  FILE* f = fopen(SHM_IMAGE_FILE, "w");
  if (!f) throw std::runtime_error("shm_backend: cannot create image segment");
  if (ftruncate(fileno(f), sizeof(ImageData)) != 0)
    throw std::runtime_error("shm_backend: ftruncate image failed");
  fclose(f);

  f = fopen(SHM_IMAGE_FILE, "r+");
  if (!f) throw std::runtime_error("shm_backend: cannot reopen image segment");
  auto* ptr = reinterpret_cast<ImageData*>(
      mmap(nullptr, sizeof(ImageData), PROT_READ | PROT_WRITE,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) throw std::runtime_error("shm_backend: mmap image failed");
  fclose(f);
  return ptr;
}

inline ImageData* open_image_shm(bool write = false) {
  FILE* f = fopen(SHM_IMAGE_FILE, write ? "r+" : "r");
  if (!f) return nullptr;
  auto* ptr = reinterpret_cast<ImageData*>(
      mmap(nullptr, sizeof(ImageData),
           write ? (PROT_READ | PROT_WRITE) : PROT_READ,
           MAP_SHARED, fileno(f), 0));
  if (ptr == MAP_FAILED) { fclose(f); return nullptr; }
  fclose(f);
  return ptr;
}

// ---------------------------------------------------------------------------
// Common helpers
// ---------------------------------------------------------------------------

/** Zero-initialize a struct without bumping the seqlock. */
template<typename T>
inline void zero_struct(T* ptr) {
  memset(ptr, 0, sizeof(T));
}
