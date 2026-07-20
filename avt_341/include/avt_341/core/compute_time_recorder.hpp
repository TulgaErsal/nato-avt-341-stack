#ifndef COMPUTE_TIME_RECORDER_HPP
#define COMPUTE_TIME_RECORDER_HPP

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "avt_341/core/running_stats.hpp"
#include "avt_341/node/node_proxy.h"
#include "avt_341/node/ros_types.h"

namespace avt_341::core {

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
    void Configure(const std::string& section_id, const RunningStatsConfig& config);

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
    /// section mean meets its configured threshold condition.
    void AddSample(const std::string& section_id, double duration_seconds);

    /// Current statistics of a section, or nullopt if the section is unknown.
    std::optional<StatsSnapshot> GetStats(const std::string& section_id);

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
    std::map<std::string, RunningStats> sections_;
    std::map<std::string, double> pending_start_seconds_;

    node::Publisher<msg::ComputeTimeArray>::SharedPtr pub_;
};

}

#endif //COMPUTE_TIME_RECORDER_HPP
