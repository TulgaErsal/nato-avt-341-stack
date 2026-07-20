#ifndef COMPUTE_TIME_RECORDER_HPP
#define COMPUTE_TIME_RECORDER_HPP

#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"

namespace avt_341::core {

/**
 * Statistics configuration for a single code section.
 * Defaults mirror the avt_341_msgs/ComputeTime message defaults (no windowing).
 */
struct SectionConfig {
    /// If > 0, only the last window_num_samples samples are used for statistics.
    int window_num_samples = -1;
    /// If > 0 (and window_num_samples <= 0), only samples within the last window_time seconds are used.
    double window_time = -1.0;
    /// If > 0, a warning is logged (throttled) when the section mean compute time exceeds this many seconds.
    double warning_threshold = -1.0;
    /// Published for parent sections synthesized from child sections; set to false on a
    /// section id to opt that section out of publisher-side parent synthesis.
    bool auto_parent_stats = true;
};

/// Snapshot of the current statistics of a code section.
struct SectionStatsSnapshot {
    double mean = 0.0;
    double std_dev = 0.0;
    std::size_t count = 0;
};

/**
 * Records compute time samples for a single code section and maintains running
 * mean / standard deviation according to the configured windowing mode.
 */
class SectionTimeStats {

public:
    explicit SectionTimeStats(const SectionConfig& config = {});

    void AddSample(double value_seconds, double now_seconds);

    /// Non-const since time-based windows evict expired samples on read.
    SectionStatsSnapshot GetStats(double now_seconds);

    const SectionConfig& GetConfig() const { return config_; }

    /// Returns true at most once per kWarningLogPeriod seconds; used to throttle warning logs.
    bool ShouldLogWarning(double now_seconds);

    static constexpr double kWarningLogPeriod = 1.0;

private:
    void Evict(double now_seconds);
    bool IsWindowed() const { return config_.window_num_samples > 0 || config_.window_time > 0.0; }

    SectionConfig config_;

    // Windowed modes: samples stored as (steady clock end time, value) with running sums
    std::deque<std::pair<double, double>> samples_;
    double sum_ = 0.0;
    double sum_sq_ = 0.0;

    // Unwindowed mode: Welford accumulators, no sample storage
    std::size_t count_ = 0;
    double mean_ = 0.0;
    double m2_ = 0.0;

    double last_warning_seconds_ = std::numeric_limits<double>::lowest();
};

class ComputeTimeRecorder;

/**
 * Recording scope returned by ComputeTimeRecorder::RecordScope.
 * Adds a compute time sample for its section when destroyed (exits scope), unless cancelled.
 */
class [[nodiscard]] ScopedRecording {

public:
    ScopedRecording(ScopedRecording&& other) noexcept;
    ScopedRecording(const ScopedRecording&) = delete;
    ScopedRecording& operator=(const ScopedRecording&) = delete;
    ScopedRecording& operator=(ScopedRecording&&) = delete;
    ~ScopedRecording();

    /// Discard this recording; no sample is added on destruction.
    void Cancel();

private:
    friend class ComputeTimeRecorder;
    ScopedRecording(ComputeTimeRecorder* recorder, std::string section_id, double start_seconds);

    ComputeTimeRecorder* recorder_;
    std::string section_id_;
    double start_seconds_;
};

/**
 * Records compute times of multiple code sections, each identified by a string id.
 * Ids may be hierarchical using a slash delimiter.
 */
class ComputeTimeRecorder {

public:
    ComputeTimeRecorder(const std::shared_ptr<node::NodeProxy>& node, const std::string& tag);

    /// Set the statistics properties of a section. Discards any samples already
    /// recorded for the section. Sections recorded without prior configuration
    /// use default (unwindowed) settings.
    void Configure(const std::string& section_id, const SectionConfig& config);

    /// Start recording the enclosing scope; the sample is added when the returned
    /// object goes out of scope.
    [[nodiscard]] ScopedRecording RecordScope(const std::string& section_id);

    /// Start a recording for a section without a scope object; the sample is added
    /// by the matching Stop call. Calling Start again before Stop restarts the timing.
    void Start(const std::string& section_id);

    /// Finish a recording started with Start and add the compute time sample.
    /// Logs a warning and adds nothing if no Start is pending for the section.
    void Stop(const std::string& section_id);

    /// Add a completed sample directly. Creates the section with default
    /// configuration if it does not exist yet. Logs a throttled warning if the
    /// section mean exceeds its configured warning_threshold.
    void AddSample(const std::string& section_id, double duration_seconds);

    /// Current statistics of a section, or nullopt if the section is unknown.
    std::optional<SectionStatsSnapshot> GetStats(const std::string& section_id);

    /// Publish the current summary of all sections with recorded samples,
    /// synthesizing entries for parent ids that were never recorded directly.
    /// No-op if no section has any samples.
    void PublishSummary();

    /// Monotonic steady clock time in seconds.
    static double NowSeconds();

    /// "<node namespace>/<node name>" tag helper for recorder construction.
    static std::string MakeNodeTag(const std::shared_ptr<node::NodeProxy>& node);

private:
    std::shared_ptr<node::NodeProxy> node_;
    std::string tag_;

    std::mutex mutex_;
    std::map<std::string, SectionTimeStats> sections_;
    std::map<std::string, double> pending_start_seconds_;

    node::Publisher<msg::ComputeTimeArray>::SharedPtr pub_;
};

}

#endif //COMPUTE_TIME_RECORDER_HPP
