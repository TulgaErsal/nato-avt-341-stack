#include "avt_341/core/monitoring.hpp"

namespace avt_341::core {
    WindowedMean::WindowedMean(const int N_size) :
        buffer_(std::queue<double>::container_type(N_size, 0.0)),
        mean_(0.0) {
    }

    double WindowedMean::GetMean() const {
        return mean_;
    }

    void WindowedMean::AddSample(const double value) {
        const auto N = static_cast<double>(buffer_.size());
        mean_ = (mean_ * N - buffer_.front() + value) / N;
        buffer_.pop();
        buffer_.push(value);
    }
}
