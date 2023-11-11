#pragma once

#include "Hungarian.h"
#include "map"
#include "stack"
#include "tracker.hpp"

/// Data structure to manage allocating tracker ids
class TrackerManager {
    /// Stack of ids to recycle
    std::stack<uint64_t> free_ids;
    /// Contiguous map of trackers. Holes in ids are tracked by free_ids, but these holes are erased in the map.
    /// This allows us to iterate over the trackers easily.
    std::map<uint64_t, Tracker> trackers;
    /// Max allowed missed frames before deallocation
    uint64_t max_missed_frames;

public:
    explicit TrackerManager(uint64_t max_missed_frames);

    /// Allocates a new tracker
    void add_tracker(cv::Point3f inital_point);

    /// Predicts the state of all tracks at some timestamp
    std::map<uint64_t, cv::Mat> predict_all(double stamp);

    /// Removes all trackers over the max frame count
    void prune();

    /// Corrects the given tracker with a measurement
    void correct_tracker(uint64_t id, const cv::Point3f& measurement);

    /// Returns all (id, state) of the trackers
    [[nodiscard]] std::vector<std::pair<uint64_t, cv::Point3f>> get_all_states() const;
};

/// Multi object tracking implementation
class MOT {
    TrackerManager tracks;
    HungarianAlgorithm solver{};
    /// Largest distance between detection and track for it to be considered a match
    double max_cost;

public:
    explicit MOT(uint64_t max_missed_frames, double max_cost);

    /// Continuously filters detections over time via tracking. Returns (id, state).
    std::vector<std::pair<uint64_t, cv::Point3f>> filter(const std::vector<cv::Point3f>& detections, double stamp);
};

template <typename TK, typename TV>
std::vector<TK> extract_keys(std::map<TK, TV> const& input_map) {
    std::vector<TK> retval;
    for (auto const& element : input_map) {
        retval.push_back(element.first);
    }
    return retval;
}