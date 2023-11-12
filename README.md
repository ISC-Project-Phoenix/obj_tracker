# obj_tracker
From package '[obj_tracker](https://github.com/ISC-Project-Phoenix/obj_tracker)'
# File
`./src/ObjTrackerNode.cpp`

## Summary 
 Generic Multiple Object Tracker (MOT) node for ROS2. Obj_tracker tracks PoseArrays over time, both smoothing the pose
and maintaining indentity. It does this by predicting with Kalman filters, then associating tracks with detections
by solving the resulting assignment problem.

## Topics

### Publishes
- `/tracks`: Array of Filtered poses

### Subscribes
- `/object_poses`: Array of Poses to filter

## Params
- `max_frames_missed`: Number of updates a tracker is allowed to predict without being matched to a detection.
Smaller numbers will be vulerable to flaky detections, larger values can stick around too long after detection leaves.
- `max_dist`: Max distince in meters allowed between a track and detection before they are considered to not be associated.
This will need to be larger for data with high amplitude in its noise, but that risks swapping tracks between detections.

- `prediction_cov`: Scalar used for the diagonal of the prediction covarience matrix of the kalman filters. Lower values weight the filter
more towards predictions.

- `measure_cov`: Scalar used for the diagonal of the measurement covarience matrix of the kalman filters. Lower values weight the filter
more towards measurements.

- `inital_vx`: Scalar used for the inital x velocity state in the kalman filters. Use this to bias the filters inital predictions to
follow object motion if you are moving forward or backwards.

- `test_latency`: Outputs the latency of the node as logs if true.

- `visualize_ids`: Publishes id markers in rviz if true.


