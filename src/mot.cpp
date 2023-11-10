#include "obj_tracker/mot.hpp"

TrackerManager::TrackerManager(uint64_t max_missed_frames) : max_missed_frames(max_missed_frames) {}

void TrackerManager::add_tracker(cv::Point3f inital_point) {
    uint64_t id;

    // If no ids to recycle, then allocate next highest id
    if (this->free_ids.empty()) {
        id = this->trackers.size();
    } else {
        id = this->free_ids.top();
        this->free_ids.pop();
    }

    this->trackers.try_emplace(id, id, inital_point);
}

std::map<uint64_t, cv::Mat> TrackerManager::predict_all(double stamp) {
    std::map<uint64_t, cv::Mat> pred;

    for (auto& [id, track] : this->trackers) {
        auto p = track.predict(stamp);

        pred.try_emplace(id, p);
    }

    return pred;
}

void TrackerManager::prune() {
    std::vector<uint64_t> to_remove;

    for (auto& [id, tracker] : this->trackers) {
        if (tracker.get_missed_frames() > this->max_missed_frames) {
            // Deallocate later to avoid breaking iter
            to_remove.push_back(id);
        }
    }

    for (auto id : to_remove) {
        this->trackers.erase(id);
        this->free_ids.push(id);
    }
}
void TrackerManager::correct_tracker(uint64_t id, const cv::Point3f& measurement) {
    this->trackers.at(id).correct(measurement);
}
std::vector<std::pair<uint64_t, cv::Point3f>> TrackerManager::get_all_states() const {
    std::vector<std::pair<uint64_t, cv::Point3f>> states;

    for (auto& [id, tracker] : this->trackers) {
        auto state = tracker.get_state();
        cv::Point3f state_v{state.at<float>(0), state.at<float>(1), state.at<float>(2)};

        states.emplace_back(id, state_v);
    }

    return states;
}

MOT::MOT(uint64_t max_missed_frames, double max_cost, rclcpp::Node& node)
    : tracks(max_missed_frames), max_cost(max_cost), node(node) {}

std::vector<std::pair<uint64_t, cv::Point3f>> MOT::filter(const std::vector<cv::Point3f>& detections, double stamp) {
    // Forward predict all tracks
    auto pred = this->tracks.predict_all(stamp);

    // TODO if pred and detec are diff sizes, we may need to pad

    // Build C_ij cost matrix as dist between each track and detection
    std::vector<std::vector<double>> cost{};
    for (auto& [i, state] : pred) {
        // Extract point from state
        cv::Point3f state_v{state.at<float>(0), state.at<float>(1), state.at<float>(2)};

        std::vector<double> track_to_detect;
        for (auto& det : detections) {
            auto dist = cv::norm(state_v - det);
            track_to_detect.push_back(dist);
        }

        cost.push_back(track_to_detect);
    }

    // Solve the assignment problem between tracks and detections
    std::vector<int> assign{};
    if (!pred.empty()) {
        this->solver.Solve(cost, assign);
    }

    // Find all missed detections
    std::vector<int> skip_set{}; //TODO this definitely doesnt work
    for (uint64_t i = 0; i < assign.size(); ++i) {
        auto paired_detect = assign[i];
        auto paired_cost = cost[i][paired_detect];

        if (paired_cost > this->max_cost) {
            RCLCPP_INFO(node.get_logger(), "Skipping: %lu", i);
            skip_set.push_back((int)i);
        }
    }

    // Map index of assignment vector to actual id
    auto assign_to_id = extract_keys(pred);

    // Correct trackers with assigned detections
    for (uint64_t i = 0; i < assign.size(); ++i) {
        // Skip if tracker missed a detection
        if (std::find(skip_set.begin(), skip_set.end(), i) != skip_set.end()) {
            continue;
        }

        RCLCPP_INFO(node.get_logger(), "Correcting: %lu", i);
        this->tracks.correct_tracker(assign_to_id[i], detections[assign[i]]);
    }

    // Allocate new tracks for unmatched detections
    for (uint64_t j = 0; j < detections.size(); ++j) {
        bool new_track = true;

        // Try to find detection in assignment - skip
        for (uint64_t i = 0; i < assign.size(); ++i) {
            if (std::find(skip_set.begin(), skip_set.end(), i) != skip_set.end()) {
                continue;
            }

            if ((uint64_t)assign[i] == j) {
                new_track = false;
                break;
            }
        }

        if (new_track) {
            RCLCPP_INFO(node.get_logger(), "Adding new track!");
            this->tracks.add_tracker(detections[j]);
        }
    }

    // Remove invalidated trackers
    this->tracks.prune();

    // Output states of all tracks
    return this->tracks.get_all_states();
}
