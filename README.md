# ros2-hand-obstacle-publisher

**Real-time human hand → live MoveIt2 collision object.**

A single-purpose ROS2 package that turns pixel-space human detections into
3D obstacles in the MoveIt2 planning scene — so your robot arm treats a
human hand (or whole arm) as a live obstacle and replans around it,
no safety cage required.

Extracted from a Human-Robot Collaboration capstone project validated on a
physical UR5e ([full project & demo videos](https://github.com/Ahmedhazemm29/Human-Robot-Collaboration---Industrial-Robotics-Project)).

```
  camera ──► your detector ──► /hand_bbox ──► hand_obstacle_publisher ──► /planning_scene ──► MoveIt2
             (MediaPipe, YOLO,    (pixels)         (this package)            (3D collision        replans
              OpenPose, ...)                                                    objects)         around the
                                                                                                   human
```

## What it does

| | |
|---|---|
| **Input** | `/hand_bbox` (`geometry_msgs/PolygonStamped`) — hand landmark/bbox points in **pixel coordinates** from any detector |
| **Output** | `/planning_scene` (`moveit_msgs/PlanningScene`) — atomic collision-object diffs at 20 Hz |
| **3D from depth** | Samples the depth image inside the detection bbox (10th percentile = closest surface), deprojects with camera intrinsics, TF-transforms to the robot's world frame |
| **No depth camera?** | Webcam fallback mode maps pixels onto a configured workspace/table volume — perfect for simulation and development |
| **Stale handling** | Detection lost → obstacle removed after a timeout (no ghost obstacles freezing your robot) |
| **Atomic updates** | Adds and removals ship in a single planning-scene diff, so the scene never contains stale duplicates |

### Bonus: full-body mode

With `tracking_mode:=body`, the node consumes 33 MediaPipe-Pose-style
landmarks on `/body_landmarks` (`geometry_msgs/PoseArray`, pixel coords,
visibility in `position.z`) and publishes a multi-part human model instead
of one box: **cylinders for upper arms and forearms, spheres for hands and
head**, with per-landmark depth sampling and visibility gating.

## Quick start

```bash
cd ~/ros2_ws/src
git clone https://github.com/Ahmedhazemm29/ros2-hand-obstacle-publisher.git
cd ~/ros2_ws && colcon build --packages-select hand_obstacle_publisher
source install/setup.bash

# hand mode (default): single box from /hand_bbox
ros2 run hand_obstacle_publisher hand_obstacle_publisher

# full-body mode: multi-part model from /body_landmarks
ros2 run hand_obstacle_publisher hand_obstacle_publisher --ros-args -p tracking_mode:=body
```

Your detector only needs to publish pixel-space points:

```python
# any detector → /hand_bbox (PolygonStamped, one Point32 per landmark, in pixels)
msg = PolygonStamped()
msg.polygon.points = [Point32(x=u_px, y=v_px) for (u_px, v_px) in detections]
```

## Message contract

**`/hand_bbox` — geometry_msgs/PolygonStamped (hand mode)**
Each `polygon.points[i]` is one detected landmark in pixels (`x` = u,
`y` = v) on a 640×480 frame. The node computes the bounding box itself, so
you can send 21 MediaPipe hand landmarks, 4 bbox corners, or anything in
between.

**`/body_landmarks` — geometry_msgs/PoseArray (body mode)**
33 poses indexed like MediaPipe Pose landmarks. `position.x/.y` = pixel
coords, `position.z` = visibility (0..1). Landmarks under the visibility
threshold are skipped, so partially occluded limbs never produce phantom
obstacles.

**`/depth/image_raw` — sensor_msgs/Image (depth mode)**
`32FC1` metres or `16UC1` millimetres, registered to the RGB frame the
detections come from.

## Configuration

Key constants at the top of
[`hand_obstacle_publisher/hand_to_collision.py`](hand_obstacle_publisher/hand_to_collision.py)
(ROS-parameterising these is on the roadmap):

| Constant | Meaning |
|---|---|
| `WEBCAM_MODE` | `True` = fixed-depth webcam mapping, `False` = depth camera + TF |
| `BASE_FRAME` / `CAMERA_FRAME` | TF frames for the world and the camera optical frame |
| `KINECT_FX/FY/CX/CY` | Camera intrinsics used for deprojection |
| `TABLE_*` | Workspace bounds the obstacle is clamped to (webcam mode) |
| `STALE_TIMEOUT` | Seconds without detections before the obstacle is removed |
| `DEPTH_EMA_ALPHA` | Depth smoothing factor (lower = smoother) |
| `UPPER_ARM_R`, `FOREARM_R`, `HAND_R`, `HEAD_R` | Body-mode primitive radii |
| `VIS_THRESH` | Body-mode landmark visibility threshold |

ROS parameter: `tracking_mode` — `"hand"` (default) or `"body"`.

## Tests

The geometry and scene-diff logic is covered by a dependency-free test
suite (ROS message classes are stubbed when ROS2 is not sourced):

```bash
python3 test/test_logic.py     # plain script — runs anywhere
# or, in a ROS2 workspace:
colcon test --packages-select hand_obstacle_publisher
```

Covered: pixel→world mapping bounds, depth-percentile box construction
inputs, quaternion alignment for body capsules, hidden-limb gating,
atomic add/remove diffs, stale-removal bookkeeping.

## Requirements

- ROS2 Humble (or newer) with MoveIt2
- `cv_bridge`, `tf2_ros`, `numpy`
- A detector publishing `/hand_bbox` (e.g. MediaPipe Hands) — not included,
  any pixel-space detector works

## License

MIT — see [LICENSE](LICENSE).
