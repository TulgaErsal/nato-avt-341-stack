#include <dlfcn.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <new>

#include "avt_341_nav/core/persistent_async_worker.hpp"

using avt_341_nav::core::PersistentAsyncWorker;

namespace {
std::atomic<int> condition_variable_ctor_delay_us{0};
}

// Overrides libstdc++'s std::condition_variable constructor so the test can slow it down.
extern "C" void _ZNSt18condition_variableC1Ev(void* self) {
  using Ctor = void (*)(void*);
  static const Ctor real_ctor = reinterpret_cast<Ctor>(dlsym(RTLD_NEXT, "_ZNSt18condition_variableC1Ev"));
  usleep(condition_variable_ctor_delay_us.load());
  real_ctor(self);
}

TEST(PersistentAsyncWorkerTest, RunsJobWhenConstructionIsSlowerThanThreadStart) {
  alignas(PersistentAsyncWorker) static char storage[sizeof(PersistentAsyncWorker)];

  condition_variable_ctor_delay_us = 50000;
  auto* worker = new (storage) PersistentAsyncWorker();
  condition_variable_ctor_delay_us = 0;

  auto result = worker->Submit([]() { return 42; });

  // On failure the worker is left undestroyed: its destructor would join a thread that never wakes.
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_EQ(result.get(), 42);
  worker->~PersistentAsyncWorker();
}
