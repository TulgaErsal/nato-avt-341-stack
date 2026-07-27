#include "avt_341/core/compute_time_recorder.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <vector>
#include "avt_341_msgs/msg/compute_time.hpp"
#include "avt_341_msgs/msg/compute_time_array.hpp"

namespace avt_341::core {

namespace {

const char* COMPUTE_TIMES_TOPIC = "avt_341/compute_times";

std::string ParentId(const std::string& section_id) {
    const auto pos = section_id.find_last_of('/');
    return pos == std::string::npos ? std::string() : section_id.substr(0, pos);
}

bool IsDirectChild(const std::string& parent_id, const std::string& section_id) {
    return section_id.size() > parent_id.size() + 1
        && section_id.compare(0, parent_id.size(), parent_id) == 0
        && section_id[parent_id.size()] == '/'
        && section_id.find('/', parent_id.size() + 1) == std::string::npos;
}

// Working entry used while building the summary message
struct SummaryEntry {
    double mean = 0.0;
    double variance = 0.0;
    int window_num_samples = -1;
    double window_time = -1.0;
    double warning_threshold = -1.0;
    bool synthesized = false;
};

}

// ScopedRecording
// ---------------------------------------------------------------------------------------------------------------

ScopedRecording::ScopedRecording(ComputeTimeRecorder* recorder, std::string section_id, const double start_seconds)
    : recorder_(recorder), section_id_(std::move(section_id)), start_seconds_(start_seconds) {
}

ScopedRecording::ScopedRecording(ScopedRecording&& other) noexcept
    : recorder_(other.recorder_), section_id_(std::move(other.section_id_)), start_seconds_(other.start_seconds_) {
    other.recorder_ = nullptr;
}

ScopedRecording::~ScopedRecording() {
    if (recorder_ != nullptr) {
        recorder_->AddSample(section_id_, ComputeTimeRecorder::NowSeconds() - start_seconds_);
    }
}

void ScopedRecording::Cancel() {
    recorder_ = nullptr;
}

// ComputeTimeRecorder
// ---------------------------------------------------------------------------------------------------------------

ComputeTimeRecorder::ComputeTimeRecorder(const rclcpp::Node::SharedPtr& node, const std::string& tag)
    : node_(node), tag_(tag) {
    pub_ = node_->create_publisher<avt_341_msgs::msg::ComputeTimeArray>(COMPUTE_TIMES_TOPIC, 10);
}

void ComputeTimeRecorder::Configure(const std::string& section_id, const RunningStatsConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    sections_.insert_or_assign(section_id, RunningStats(config));
}

ScopedRecording ComputeTimeRecorder::RecordScope(const std::string& section_id) {
    return ScopedRecording(this, section_id, NowSeconds());
}

void ComputeTimeRecorder::Start(const std::string& section_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_start_seconds_[section_id] = NowSeconds();
}

void ComputeTimeRecorder::Stop(const std::string& section_id) {
    const double end_seconds = NowSeconds();

    double start_seconds;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = pending_start_seconds_.find(section_id);
        if (it == pending_start_seconds_.end()) {
            RCLCPP_WARN(node_->get_logger(), "Stop called for section %s without a matching Start.", section_id.c_str());
            return;
        }
        start_seconds = it->second;
        pending_start_seconds_.erase(it);
    }

    AddSample(section_id, end_seconds - start_seconds);
}

void ComputeTimeRecorder::AddSample(const std::string& section_id, const double duration_seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now_seconds = NowSeconds();
    RunningStats& stats = sections_.try_emplace(section_id).first->second;
    stats.AddSample(duration_seconds, now_seconds);

    const RunningStatsConfig& config = stats.GetConfig();
    if (config.threshold_check <= 0.0) {
        return;
    }

    const StatsSnapshot snapshot = stats.GetStats(now_seconds);
    if (config.IsThresholdMet(snapshot.mean) && stats.ShouldLogWarning(now_seconds)) {
        RCLCPP_WARN(node_->get_logger(), "%s took %.2f ms (%s %.2f ms warning threshold).", section_id.c_str(), snapshot.mean * 1e3, config.threshold_greater_than ? ">" : "<", config.threshold_check * 1e3);
    }
}

std::optional<StatsSnapshot> ComputeTimeRecorder::GetStats(const std::string& section_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = sections_.find(section_id);
    if (it == sections_.end()) {
        return std::nullopt;
    }
    return it->second.GetStats(NowSeconds());
}

void ComputeTimeRecorder::PublishSummary() {
    avt_341_msgs::msg::ComputeTimeArray summary_msg;
    summary_msg.header.stamp = node_->now();
    summary_msg.header.frame_id = "";
    summary_msg.tag = tag_;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now_seconds = NowSeconds();

        // Sections with direct samples
        std::map<std::string, SummaryEntry> entries;
        for (auto& [section_id, stats] : sections_) {
            const StatsSnapshot snapshot = stats.GetStats(now_seconds);
            if (snapshot.count == 0) {
                continue;
            }
            SummaryEntry entry;
            entry.mean = snapshot.mean;
            entry.variance = snapshot.std_dev * snapshot.std_dev;
            entry.window_num_samples = stats.GetConfig().window_num_samples;
            entry.window_time = stats.GetConfig().window_time;
            entry.warning_threshold = stats.GetConfig().threshold_check;
            entries.emplace(section_id, entry);
        }

        if (entries.empty()) {
            return;
        }

        // Ancestor ids implied by the recorded section ids but never recorded directly,
        // excluding those explicitly configured with auto_parent_stats = false
        std::set<std::string> ancestor_ids;
        for (const auto& [section_id, entry] : entries) {
            for (std::string parent = ParentId(section_id); !parent.empty(); parent = ParentId(parent)) {
                const auto config_it = sections_.find(parent);
                const bool synthesis_enabled = config_it == sections_.end() || config_it->second.GetConfig().auto_parent_stats;
                if (entries.find(parent) == entries.end() && synthesis_enabled) {
                    ancestor_ids.insert(parent);
                }
            }
        }

        // Synthesize ancestors deepest-first so multi-level hierarchies aggregate bottom-up.
        // Stats are combined from direct children: summed means/variances, smallest positive window values.
        std::vector<std::string> sorted_ancestors(ancestor_ids.begin(), ancestor_ids.end());
        std::sort(sorted_ancestors.begin(), sorted_ancestors.end(),
            [](const std::string& a, const std::string& b) {
                return std::count(a.begin(), a.end(), '/') > std::count(b.begin(), b.end(), '/');
            });

        for (const auto& ancestor_id : sorted_ancestors) {
            SummaryEntry parent_entry;
            parent_entry.synthesized = true;
            bool has_children = false;
            for (const auto& [section_id, entry] : entries) {
                if (!IsDirectChild(ancestor_id, section_id)) {
                    continue;
                }
                has_children = true;
                parent_entry.mean += entry.mean;
                parent_entry.variance += entry.variance;
                if (entry.window_num_samples > 0
                    && (parent_entry.window_num_samples <= 0 || entry.window_num_samples < parent_entry.window_num_samples)) {
                    parent_entry.window_num_samples = entry.window_num_samples;
                }
                if (entry.window_time > 0.0
                    && (parent_entry.window_time <= 0.0 || entry.window_time < parent_entry.window_time)) {
                    parent_entry.window_time = entry.window_time;
                }
            }
            if (has_children) {
                entries.emplace(ancestor_id, parent_entry);
            }
        }

        summary_msg.compute_times.reserve(entries.size());
        for (const auto& [section_id, entry] : entries) {
            avt_341_msgs::msg::ComputeTime compute_time_msg;
            compute_time_msg.section_id = section_id;
            compute_time_msg.time = static_cast<float>(entry.mean);
            compute_time_msg.time_std = static_cast<float>(std::sqrt(entry.variance));
            compute_time_msg.window_num_samples = entry.window_num_samples;
            compute_time_msg.window_time = static_cast<float>(entry.window_time);
            compute_time_msg.warning_threshold = static_cast<float>(entry.warning_threshold);
            compute_time_msg.auto_parent_stats = entry.synthesized;
            summary_msg.compute_times.push_back(compute_time_msg);
        }
    }

    pub_->publish(summary_msg);
}

double ComputeTimeRecorder::NowSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string ComputeTimeRecorder::MakeNodeTag(const rclcpp::Node::SharedPtr& node) {
    std::string ns = node->get_namespace();
    if (ns.empty() || ns.back() != '/') {
        ns += '/';
    }
    return ns + node->get_name();
}

}
