#pragma once

#include "cstdint"
#include "opencv2/video/tracking.hpp"
#include "optional"

class Tracker {
    uint64_t id;
    cv::KalmanFilter filter{};
    std::optional<double> last_stamp;
    uint8_t missed_frames = 0;

    /// Updates the time between measurements
    void update_dt(double stamp);

public:
    Tracker(uint64_t id, const cv::Point3f& inital_Point);

    /// Predicts the next location of the track
    cv::Mat predict(double stamp);

    /// Corrects the filter
    cv::Mat correct(const cv::Point3f& point);
};