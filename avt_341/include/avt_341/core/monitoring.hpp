#include <queue>

#ifndef MONITORING_HPP
#define MONITORING_HPP

namespace avt_341::core {

class WindowedMean {

public:
    WindowedMean() = default;
    explicit WindowedMean(const int N_size);
    double GetMean() const;
    void AddSample(const double value);

private:

    // Currently used instead of eigen circular buffer to avoid dependency
    std::queue<double> buffer_;

    double mean_;
};

}

#endif //MONITORING_HPP
