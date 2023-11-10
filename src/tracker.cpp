#include "obj_tracker/tracker.hpp"

Tracker::Tracker(uint64_t id, const cv::Point3f& inital_Point) {
    this->id = id;
    // state: (x,y,z,vx,vy,vz) measure: (x,y,z)
    this->filter.init(6, 3);

    // clang-format off

    // Maps 3d points to the state
    cv::Mat_<float> measure = (cv::Mat_<float>(3, 6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0);

    // Predicts the next location of tracks
    // This assumes linear motion, and velocities are estimated
    cv::Mat_<float> trans = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0, // x
        0, 1, 0, 0, 1, 0, // y
        0, 0, 1, 0, 0, 1, // z
        0, 0, 0, 1, 0, 0, // vx
        0, 0, 0, 0, 1, 0, // vy
        0, 0, 0, 0, 0, 1); // vz

    // Prediction covariance TODO tune
    cv::Mat_<float> proc_noise = cv::Mat::eye(6, 6, CV_32F) * 0.01;

    cv::Mat noise_pre = cv::Mat::eye(6, 6, CV_32F);
    cv::Mat state_pre = cv::Mat::zeros(6, 1, CV_32F);
    cv::Mat state_post = cv::Mat::zeros(6, 1, CV_32F);

    // clang-format on

    // Set initial values
    state_pre.at<float>(0) = inital_Point.x;
    state_pre.at<float>(1) = inital_Point.y;
    state_pre.at<float>(2) = inital_Point.z;

    state_post.at<float>(0) = inital_Point.x;
    state_post.at<float>(1) = inital_Point.y;
    state_post.at<float>(2) = inital_Point.z;

    this->filter.measurementMatrix = measure;
    this->filter.transitionMatrix = trans;
    this->filter.processNoiseCov = proc_noise;
    this->filter.statePost = state_post;
    this->filter.statePre = state_pre;
    this->filter.errorCovPre = noise_pre;
    cv::setIdentity(filter.measurementNoiseCov, cv::Scalar(1e-1));
}

void Tracker::update_dt(double stamp) {
    float dt = 0.01;
    if (!last_stamp) {
        last_stamp = stamp;
    } else {
        dt = (float)(stamp - *last_stamp);
        last_stamp = stamp;
    }

    // Update dt for each x, y, z
    this->filter.transitionMatrix.at<float>(0, 3) = dt;
    this->filter.transitionMatrix.at<float>(1, 4) = dt;
    this->filter.transitionMatrix.at<float>(2, 5) = dt;
}

cv::Mat Tracker::predict(double stamp) {
    this->update_dt(stamp);
    this->missed_frames++;

    return this->filter.predict();
}

cv::Mat Tracker::correct(const cv::Point3f& point) {
    this->missed_frames = 0;
    return this->filter.correct((cv::Mat_<float>(3, 1) << point.x, point.y, point.z));
}
cv::Mat Tracker::get_state() const { return filter.statePost; }

uint64_t Tracker::get_missed_frames() const { return this->missed_frames; }
