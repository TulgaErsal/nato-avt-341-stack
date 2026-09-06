#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

namespace avt_341_nav {
namespace core {

class PersistentAsyncWorker {
public:
  PersistentAsyncWorker() : thread_(&PersistentAsyncWorker::Run, this) {}

  ~PersistentAsyncWorker() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    thread_.join();
  }

  PersistentAsyncWorker(const PersistentAsyncWorker&) = delete;
  PersistentAsyncWorker& operator=(const PersistentAsyncWorker&) = delete;

  template <typename Fn>
  auto Submit(Fn&& fn) -> std::future<decltype(fn())> {
    using Result = decltype(fn());
    auto task = std::make_shared<std::packaged_task<Result()>>(std::forward<Fn>(fn));
    std::future<Result> future = task->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      job_ = [task]() { (*task)(); };
    }
    cv_.notify_one();
    return future;
  }

private:
  void Run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      cv_.wait(lock, [this]() { return stop_ || job_; });
      if (stop_ && !job_) {
        return;
      }
      auto job = std::move(job_);
      job_ = nullptr;
      lock.unlock();
      job();
      lock.lock();
    }
  }

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::function<void()> job_;
  bool stop_ = false;
};

}  // namespace core
}  // namespace avt_341_nav
