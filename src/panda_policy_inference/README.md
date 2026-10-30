# GPT discrete Cartesian policy

For the single-arm Show-Harness zero-shot reproduction, including multi-camera
inputs, adaptive steps, rotations, gripper control and recovery, see
[SHOW_HARNESS.md](SHOW_HARNESS.md). This page describes the translation baseline.

The node implements `observation → one GPT action → bounded Cartesian step →
measured settling → new observation`. The arm controller holds position while
GPT reasons. GPT is a slow action selector; MoveIt and
the Panda trajectory controller plan and execute each accepted step. Numerical
outputs are possible from GPT, but discrete labels give
this experiment a small, deterministic command interface.

The original prototype stringified the ROS image (so GPT did not receive image
input), made blocking API calls in a subscriber callback, discarded callback
return values, called undefined parameter helpers, and had no action-to-pose
execution. This implementation sends JPEG image input and compact numerical
state through the Responses API, uses a strict JSON action schema plus local
validation, and caches only the latest observation in the subscriber callback.
It retains the prototype's model name; access and supported reasoning settings
depend on your account/model. A refusal, incomplete response, API error, invalid
decision, UNCERTAIN, or low confidence ends the run without another movement.

## Action contract

Directions are defined in `panda_link0`, not camera pixels or the tool frame.
With the default `step_size: 0.01`:

| Action | Delta position, metres |
| --- | --- |
| MOVE_FORWARD | (+0.01, 0, 0) |
| MOVE_BACKWARD | (-0.01, 0, 0) |
| MOVE_LEFT | (0, +0.01, 0) |
| MOVE_RIGHT | (0, -0.01, 0) |
| MOVE_UP | (0, 0, +0.01) |
| MOVE_DOWN | (0, 0, -0.01) |
| STOP | End run; model believes task is complete |
| UNCERTAIN | End run; insufficient information or unavailable action |

The node calculates `target = latest_measured_position + delta` once per
accepted decision. It converts Franka's measured `O_T_EE` target through the
configured `F_T_EE` tool transform to a `panda_link8` goal, then asks MoveIt for
a straight Cartesian path with collision checking enabled. MoveIt
time-parameterizes and executes that path through
`position_joint_trajectory_controller`. The startup quaternion remains fixed.
The node waits for both position error and joint velocity to remain small for
`settle_time` before another API call. Only completed steps and measured
outcomes enter the short history. An out-of-bounds or partial Cartesian path is
rejected, not clipped. This baseline node does not implement rotation or grasping.

## Setup and dry run

Use a Python environment compatible with your ROS installation. For this
workspace's ROS Noetic / Python 3.8, the requirements file selects OpenAI SDK
1.x with Responses support. It does not require a separate Pydantic schema.
For example, from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
python3 -m venv --system-site-packages /tmp/panda-gpt-venv
source /tmp/panda-gpt-venv/bin/activate
python3 -m pip install -r src/panda_policy_inference/requirements.txt
source devel/setup.bash
read -rsp 'OpenAI API key: ' OPENAI_API_KEY
export OPENAI_API_KEY
```

The current catkin workspace must have generated the observation message
(`catkin_make` if needed). `roslaunch` may use a Python interpreter
captured at build time instead of your active virtual environment; running
`python3` explicitly below avoids that ambiguity. Install the requirements in
the interpreter used by the node if you use the supplied launch file.

First edit `config/gpt_discrete_inference.yaml`: set workspace bounds for your
actual setup and describe the camera's relationship to robot-base axes in
`camera_context`. The defaults explicitly say that relationship is unknown.
For visual tasks this may correctly produce UNCERTAIN. Describe the target
unambiguously in the task. A monocular RGB image alone does not establish
metric depth; for 3D approach tasks supply calibrated geometry/depth or target
coordinates in the task/context. The current node sends RGB, not a depth map
or camera intrinsics, and does not compute camera-to-base transforms.

The package startup launch starts the RealSense camera, Franka connection,
the real-robot MoveIt stack with its position trajectory controller, and the
observation receiver:

```bash
roslaunch panda_policy_inference startup_gpt.launch robot_ip:=YOUR_ROBOT_IP
```

The receiver synchronizes **all three** topics; it publishes nothing if image,
Franka state, or gripper joint state is missing. Its output header is the camera
header. `franka_state_controller` may leave the state header's frame ID empty;
in that case the policy interprets `O_T_EE` in the configured `base_frame`, as
defined by libfranka's robot-frame convention. A non-empty conflicting state
frame is rejected.

Run one preview from the active Python environment:

```bash
rosparam load src/panda_policy_inference/config/gpt_discrete_inference.yaml /gpt_discrete_inference
python3 src/panda_policy_inference/scripts/gpt_discrete_inference.py \
  _task_description:='Move the end effector toward the red marker without touching it.' \
  _dry_run:=true
```

Or, when the launch interpreter has the SDK:

```bash
roslaunch panda_policy_inference gpt_discrete_inference.launch \
  task:='Move the end effector toward the red marker without touching it.'
```

For a calibrated fixed camera, `camera_context` can also be overridden from
the launch command. Replace the example relationships below with measurements
from the actual installation; do not copy its axis signs blindly:

```bash
roslaunch panda_policy_inference gpt_discrete_inference.launch \
  task:='Align the end effector above the red marker in the table plane.' \
  dry_run:=true \
  camera_context:='Fixed overhead camera. Measured mapping: image-up is panda_link0 +X and image-left is panda_link0 +Y. The marker is on the horizontal table plane. Use only X/Y actions for image-plane alignment; depth/Z cannot be established from this RGB image.'
```

The policy node is intentionally finite: dry run exits after one preview, and
a live run exits on STOP, UNCERTAIN, an error, or `max_decisions`. Therefore it
is not marked as a required roslaunch process. A clean node exit is an expected
policy outcome rather than a launch failure.

Dry run makes one real API call (sending the image/state and incurring API
usage), logs the proposed target, and exits. It does not initialize MoveIt or
simulate execution. It still requires fresh observations and a
stationary robot within the configured workspace. The launch file loads a flat YAML in the node's private
namespace; the old top-level `gpt_discrete_inference:` wrapper was removed.

## MoveIt live experiment

The current `startup_gpt.launch` uses
`panda_moveit_config/franka_control.launch`. Do not use MoveIt's `demo.launch`
by itself for a live run: its fake controller can report execution without
moving the robot. The policy explicitly rejects a fake controller manager and
requires `position_joint_trajectory_controller` to be running:

```bash
roslaunch panda_policy_inference startup_gpt.launch robot_ip:=YOUR_ROBOT_IP
```

Keep `load_gripper` enabled so the observation receiver still gets gripper
joint states. Add the table, fixtures, and other fixed obstacles to MoveIt's
planning scene before live operation. The RGB image sent to GPT does not
automatically become MoveIt collision geometry.

After checking the preview, bounds, and base-frame direction convention, use
the same policy command with `_dry_run:=false` (or `dry_run:=false` in
roslaunch). Only one process should command the arm controller.
Start with an unobstructed, stationary scene and a simple position task.

The policy rejects stale/unsynchronized observations, late model decisions,
pose drift during inference or planning, orientation drift, disagreement
between MoveIt and Franka frames, partial Cartesian plans, and steps outside
the box. It ends after `max_decisions` even without STOP. On an error it cancels
MoveIt motion and the trajectory controller holds its last command. MoveIt
checks its configured planning scene, IK, and joint limits, but it cannot avoid
objects that are absent or stale in that scene. These checks are not an
emergency stop. Use the robot's hardware stop and supervised operation. Model
confidence is not a safety probability, and fresh robot state cannot detect
every scene change while GPT is reasoning.

## Evaluate whether the policy helps

The node logs JSON decisions/outcomes, API duration and token usage to ROS logs
and `/gpt_discrete_inference/decision`. Record that topic together with
`/synchronized_observation` and joint trajectory/controller topics using rosbag. Start recording
before launching the policy, since it exits after a dry-run preview. Compare
task success, distance-to-target before/after each step, oscillation frequency,
UNCERTAIN rate, latency, and token usage across repeated trials. STOP is only
the model's judgment, so independently score whether the task was completed.
Tasks requiring grasping, rotation, or accurate unknown depth are outside this
six-action experiment.

Offline regression tests use real ROS messages and mocked API/MoveIt I/O;
they need no ROS master, API key, or robot:

```bash
source devel/setup.bash
python3 -m unittest discover -s src/panda_policy_inference/tests -v
```

API implementation references: [image inputs](https://developers.openai.com/api/docs/guides/images-vision)
and [Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs).
