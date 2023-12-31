#pragma once

#include "cstdint"
#include "opencv2/video/tracking.hpp"
#include "optional"

/// Tracking state of a single pose
class Tracker {
    uint64_t id;
    cv::KalmanFilter filter{};
    /// Last timestamp predicted at
    std::optional<double> last_stamp;
    /// Number of frames this tracker has predicted with no correction
    uint8_t missed_frames = 0;

    /// Updates the time between measurements
    void update_dt(double stamp);

public:
    Tracker(uint64_t id, const cv::Point3f& inital_Point, float pred_cov, float measure_cov, float inital_vx);

    /// Predicts the next location of the track.
    /// A timestamp in seconds is passed to aid in predicting motion.
    cv::Mat predict(double stamp);

    /// Corrects the filter
    cv::Mat correct(const cv::Point3f& point);

    /// Returns the current state matrix
    [[nodiscard]] cv::Mat get_state() const;

    [[nodiscard]] uint64_t get_missed_frames() const;
};