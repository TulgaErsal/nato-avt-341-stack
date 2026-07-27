#ifndef RUNNING_STATS_HPP
#define RUNNING_STATS_HPP

#include <cstddef>
#include <deque>
#include <limits>
#include <utility>

namespace avt_341_nav::core {

/**
 * Statistics configuration for a single tracked quantity.
 * Defaults disable windowing and threshold checking.
 */
struct RunningStatsConfig {
    /// If > 0, only the last window_num_samples samples are used for statistics.
    int window_num_samples = -1;
    /// If > 0 (and window_num_samples <= 0), only samples within the last window_time seconds are used.
    double window_time = -1.0;
    /// If > 0, threshold checking of the mean is enabled via IsThresholdMet.
    double threshold_check = -1.0;
    /// If true, the threshold condition is mean > threshold_check; otherwise mean < threshold_check.
    bool threshold_greater_than = true;
    /// Published for parent sections synthesized from child sections; set to false on a
    /// section id to opt that section out of publisher-side parent synthesis.
    bool auto_parent_stats = true;

    /// True if threshold checking is enabled and value meets the configured threshold condition.
    bool IsThresholdMet(double value) const;
};

/// Snapshot of the current statistics of a tracked quantity.
struct StatsSnapshot {
    double mean = 0.0;
    double std_dev = 0.0;
    std::size_t count = 0;
};

/**
 * Records samples of a scalar quantity and maintains running mean / standard
 * deviation according to the configured windowing mode.
 */
class RunningStats {

public:
    explicit RunningStats(const RunningStatsConfig& config = {});

    void AddSample(double value, double now_seconds);

    /// Non-const since time-based windows evict expired samples on read.
    StatsSnapshot GetStats(double now_seconds);

    const RunningStatsConfig& GetConfig() const { return config_; }

    /// Returns true at most once per kWarningLogPeriod seconds; used to throttle warning logs.
    bool ShouldLogWarning(double now_seconds);

    static constexpr double kWarningLogPeriod = 1.0;

private:
    void Evict(double now_seconds);
    bool IsWindowed() const { return config_.window_num_samples > 0 || config_.window_time > 0.0; }

    RunningStatsConfig config_;

    // Windowed modes: samples stored as (steady clock time, value) with running sums
    std::deque<std::pair<double, double>> samples_;
    double sum_ = 0.0;
    double sum_sq_ = 0.0;

    // Unwindowed mode: Welford accumulators, no sample storage
    std::size_t count_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;

    double last_warning_seconds_ = std::numeric_limits<double>::lowest();
};

}

#endif //RUNNING_STATS_HPP
