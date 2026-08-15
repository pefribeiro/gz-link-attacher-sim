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

**On macOS**, `gz-sim8`'s own CMake config unconditionally requires Qt5 (via `gz-gui8`), even
though this plugin has no GUI dependency of its own -- Homebrew's `qt@5` is keg-only (deprecated,
not symlinked into the default prefix), so plain `cmake -S . -B build` fails with `Could not find a
package configuration file provided by "Qt5"` unless you point `CMAKE_PREFIX_PATH` at it:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(brew --prefix qt@5)"
```

Point Gazebo at the resulting `libLinkAttacher.so`/`.dylib` via `GZ_SIM_SYSTEM_PLUGIN_PATH`, and
add `<plugin filename="LinkAttacher" name="gz_link_attacher_sim::LinkAttacher"></plugin>` to your
world's SDF.

## Prebuilt releases

Tagged releases build and attach `.so`/`.dylib` binaries for Linux (x86_64, arm64) and macOS
(arm64, x86_64) via GitHub Actions -- see the [Releases](../../releases) page. Prebuilt binaries
are tied to whatever `gz-sim8` ABI they were built against; if you're actively developing against a
patched/custom Gazebo build, building from source (above) is more reliable than a release binary.

## Using this in a plain ROS2 workspace

If Gazebo and ROS2 run on the same machine (no [vscode-gz-bridge](https://github.com/pefribeiro/vscode-gz-bridge)
involved), the idiomatic way to make this plugin discoverable isn't to check a binary into your
workspace or hardcode a path in SDF -- it's a small **vendor package**, the same pattern this
project's own ROS2 distro already uses for `gz-sim8`, `gz-transport13`, etc. (ament packages whose
`CMakeLists.txt` fetches something built elsewhere rather than committing it to git). This also
makes the plugin automatically discoverable by [vscode-gz-bridge](https://github.com/pefribeiro/vscode-gz-bridge)
if you're using it, via the `<gz_native_plugin_release>` export documented below -- no separate
configuration needed either way.

Create `gz_link_attacher_sim_vendor/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(gz_link_attacher_sim_vendor)

find_package(ament_cmake REQUIRED)

set(GZ_LINK_ATTACHER_SIM_REPO "pefribeiro/gz-link-attacher-sim")
set(GZ_LINK_ATTACHER_SIM_TAG "v0.1.0")  # pin to a specific tag, not "latest" -- avoids silently
                                          # drifting to a build against a different gz-sim8 ABI

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(_arch "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "amd64")
  set(_arch "x86_64")
else()
  message(FATAL_ERROR "unsupported CMAKE_SYSTEM_PROCESSOR '${CMAKE_SYSTEM_PROCESSOR}'")
endif()

set(_asset "libLinkAttacher-linux-${_arch}.so")
set(_url "https://github.com/${GZ_LINK_ATTACHER_SIM_REPO}/releases/download/${GZ_LINK_ATTACHER_SIM_TAG}/${_asset}")
set(_downloaded "${CMAKE_CURRENT_BINARY_DIR}/${_asset}")

if(NOT EXISTS "${_downloaded}")
  file(DOWNLOAD "${_url}" "${_downloaded}" STATUS _status TLS_VERIFY ON)
  list(GET _status 0 _code)
  if(NOT _code EQUAL 0)
    file(REMOVE "${_downloaded}")
    message(FATAL_ERROR "failed to download ${_url}")
  endif()
endif()

install(FILES "${_downloaded}" DESTINATION lib/${PROJECT_NAME} RENAME libLinkAttacher.so)
ament_package()
```

And `gz_link_attacher_sim_vendor/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>gz_link_attacher_sim_vendor</name>
  <version>0.1.0</version>
  <description>Vendor package for gz-link-attacher-sim's LinkAttacher plugin.</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <!-- Standard ros_gz_sim/gazebo_ros_pkgs convention: GazeboRosPaths.get_paths() in
         ros_gz_sim's gz_sim.launch.py scans every installed package for this export to build
         GZ_SIM_SYSTEM_PLUGIN_PATH automatically. -->
    <gazebo_ros plugin_path="${prefix}/../../lib/gz_link_attacher_sim_vendor"/>
    <!-- Not a standard ros_gz_sim export -- lets vscode-gz-bridge's gz-bridge-remote extension
         discover which release this workspace uses, so a natively-running Gazebo Sim on a
         different machine (e.g. a Mac host) can fetch the matching macOS/other-arch asset
         automatically. repo/tag must match CMakeLists.txt above. {platform}/{arch}/{ext} are
         substituted with macos|linux, arm64|x86_64, and dylib|so respectively. -->
    <gz_native_plugin_release
      repo="pefribeiro/gz-link-attacher-sim"
      tag="v0.1.0"
      asset_pattern="libLinkAttacher-{platform}-{arch}.{ext}"/>
  </export>
</package>
```

`colcon build` fetches the right Linux binary at build time (once, not at runtime) -- nothing is
committed to git. Reference the plugin by bare name in your world's SDF:

```xml
<plugin filename="LinkAttacher" name="gz_link_attacher_sim::LinkAttacher"></plugin>
```

and launch Gazebo the normal way, via `ros_gz_sim`'s `gz_sim.launch.py` -- `GazeboRosPaths.get_paths()`
picks up the `plugin_path` export automatically, same as it already does for every other
Gazebo-plugin-shipping ROS2 package.

For one-off/local testing where a whole vendor package isn't worth setting up, you can skip all of
this and reference a downloaded binary by its full path directly instead:
`<plugin filename="/full/path/to/libLinkAttacher.so" ...>` -- works, just isn't portable to a
different machine or checkout location.
