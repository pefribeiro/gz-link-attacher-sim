# gz-link-attacher-sim

A [Gazebo Sim](https://gazebosim.org/) (Harmonic / gz-sim8) System that dynamically creates and
removes a fixed joint between two named model links at runtime, via two gz-transport services:

- `/link_attacher/attach` -- `gz.msgs.StringMsg_V` request `[model1, link1, model2, link2]`,
  `gz.msgs.Boolean` reply.
- `/link_attacher/detach` -- same request shape, releases a previously-created joint.

A repeat `attach` call for the same pair reuses the existing joint rather than creating a
duplicate; an unknown model/link name returns `false` rather than erroring.

## Why this exists

This ports [`gazebo_ros_link_attacher`](https://github.com/pal-robotics/gazebo_ros_link_attacher)
(originally a Gazebo **Classic** `WorldPlugin`) to Gazebo Sim/Harmonic, which is a different
codebase with an incompatible plugin API -- not a recompile. It's deliberately **gz-transport-only,
not an embedded ROS2 node**: this plugin is meant to run alongside a natively-running Gazebo Sim in
[vscode-gz-bridge](https://github.com/pefribeiro/vscode-gz-bridge)'s architecture (Gazebo native on
a Mac host, ROS2 in a Dev Container), which relays gz-transport end-to-end across that boundary
already but does not bridge ROS2/DDS. A ROS2-facing `/attach`/`/detach` service pair (matching the
original plugin's interface) lives in a small separate adapter node, in the `gazebo_ros_link_attacher`
ROS2 package, translating ROS2 service calls into gz-transport calls to this plugin -- the same
shape `ros_gz_sim`'s own `spawn_entity`/`set_entity_pose`/`delete_entity` nodes already use.

## Mechanism

Runtime joint creation reuses gz-sim's own `components::DetachableJoint` component -- confirmed
directly against gz-sim8's actual `DetachableJoint` system source, not assumed:

```cpp
auto joint = ecm.CreateEntity();
ecm.CreateComponent(joint, components::DetachableJoint({parentLink, childLink, "fixed"}));
```

No other components are required; gz-sim's own `Physics` system does the rest. Detach is
`ecm.RequestRemoveEntity(joint)`.

The gz-transport service callback fires on gz-transport's own thread pool, not gz-sim's simulation
thread -- touching the `EntityComponentManager` there directly would race with `PreUpdate` (the
same thread-safety split [`gz_ros2_control`](https://github.com/ros-controls/gz_ros2_control) uses
between its background rclcpp executor thread and gz-sim's update thread). So the callback queues
the request and *blocks* until `PreUpdate` has actually done the ECM work and produced a result --
a synchronous producer/consumer handoff, not fire-and-forget, since the gz-transport service
contract requires the reply to be fully populated before the callback returns. In practice this
means a call is answered within one simulation step, not a meaningful delay for an occasional
attach/detach call.

## Build

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Requires `gz-sim8`, `gz-transport13`, `gz-msgs10`, `gz-plugin2` development packages --
`gz-harmonic` via [Homebrew](https://formulae.brew.sh/formula/gz-harmonic) on macOS, or via the
[OSRF apt repository](https://gazebosim.org/docs/harmonic/install_ubuntu) on Ubuntu.

Point Gazebo at the resulting `libLinkAttacher.so`/`.dylib` via `GZ_SIM_SYSTEM_PLUGIN_PATH`, and
add `<plugin filename="LinkAttacher" name="gz_link_attacher_sim::LinkAttacher"></plugin>` to your
world's SDF.

## Prebuilt releases

Tagged releases build and attach `.so`/`.dylib` binaries for Linux (x86_64, arm64) and macOS
(arm64, x86_64) via GitHub Actions -- see the [Releases](../../releases) page. Prebuilt binaries
are tied to whatever `gz-sim8` ABI they were built against; if you're actively developing against a
patched/custom Gazebo build, building from source (above) is more reliable than a release binary.
