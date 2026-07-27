#include "avt_341_nav/core/running_stats.hpp"

#include <algorithm>
#include <cmath>

namespace avt_341_nav::core {

// RunningStatsConfig
// ---------------------------------------------------------------------------------------------------------------

bool RunningStatsConfig::IsThresholdMet(const double value) const {
    if (threshold_check <= 0.0) {
        return false;
    }
    return threshold_greater_than ? value > threshold_check : value < threshold_check;
}

// RunningStats
// ---------------------------------------------------------------------------------------------------------------

RunningStats::RunningStats(const RunningStatsConfig& config) : config_(config) {
}

void RunningStats::AddSample(const double value, const double now_seconds) {
    if (!IsWindowed()) {
        count_++;
        const double delta = value - mean_;
        mean_ += delta / static_cast<double>(count_);
        m2_ += delta * (value - mean_);
        return;
    }

    samples_.emplace_back(now_seconds, value);
    sum_ += value;
    sum_sq_ += value * value;
    Evict(now_seconds);
}

void RunningStats::Evict(const double now_seconds) {
    const auto pop_front = [this]() {
        sum_ -= samples_.front().second;
        sum_sq_ -= samples_.front().second * samples_.front().second;
        samples_.pop_front();
    };

    if (config_.window_num_samples > 0) {
        while (samples_.size() > static_cast<std::size_t>(config_.window_num_samples)) {
            pop_front();
        }
    } else if (config_.window_time > 0.0) {
        while (!samples_.empty() && now_seconds - samples_.front().first > config_.window_time) {
            pop_front();
        }
    }
}

StatsSnapshot RunningStats::GetStats(const double now_seconds) {
    StatsSnapshot snapshot;

    if (!IsWindowed()) {
        snapshot.count = count_;
        snapshot.mean = mean_;
        snapshot.std_dev = count_ > 0 ? std::sqrt(m2_ / static_cast<double>(count_)) : 0.0;
        return snapshot;
    }

    Evict(now_seconds);
    snapshot.count = samples_.size();
    if (snapshot.count > 0) {
        const auto n = static_cast<double>(snapshot.count);
        snapshot.mean = sum_ / n;
        // Clamp guards against small negative values from floating point rounding
        snapshot.std_dev = std::sqrt(std::max(0.0, sum_sq_ / n - snapshot.mean * snapshot.mean));
    }
    return snapshot;
}

bool RunningStats::ShouldLogWarning(const double now_seconds) {
    if (now_seconds - last_warning_seconds_ < kWarningLogPeriod) {
        return false;
    }
    last_warning_seconds_ = now_seconds;
    return true;
}

}
