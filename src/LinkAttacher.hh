#ifndef GZ_LINK_ATTACHER_SIM_LINKATTACHER_HH_
#define GZ_LINK_ATTACHER_SIM_LINKATTACHER_HH_

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

namespace gz_link_attacher_sim
{
/// A world-scoped gz-sim System exposing gz-sim's runtime detachable-joint mechanism
/// (components::DetachableJoint) as two gz-transport services, "/link_attacher/attach" and
/// "/link_attacher/detach", each taking a StringMsg_V of exactly [model1, link1, model2, link2].
///
/// Deliberately gz-transport-only, not an embedded ROS2 node -- see this repo's README for why:
/// vscode-gz-bridge (the project this exists for) relays gz-transport end-to-end across a Dev
/// Container boundary already (including service request/reply, via its ServiceReplyProxy), but
/// does not bridge ROS2/DDS at all. A ROS2-facing service with the same original
/// gazebo_ros_link_attacher interface lives in a small adapter node instead, inside the ROS2
/// workspace this plugin doesn't need to know about.
class LinkAttacher : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

private:
  using JointKey = std::tuple<std::string, std::string, std::string, std::string>;

  /// One queued attach/detach call. The gz-transport service callback fires on gz-transport's own
  /// thread pool, not gz-sim's simulation thread -- touching the ECM there would race with
  /// PreUpdate (same thread-safety split gz_ros2_control uses between its background rclcpp
  /// executor thread and gz-sim's update thread, confirmed against its actual source). So the
  /// callback only enqueues a request here and *blocks* on `cv` until PreUpdate has actually done
  /// the ECM work and set `done`/`result` -- a plain producer/consumer handoff, not a fire-and-
  /// forget notification, since the gz-transport service contract requires the reply to be fully
  /// populated before the callback returns.
  struct PendingRequest
  {
    JointKey key;
    bool attach;
    bool done = false;
    bool result = false;
  };

  bool OnAttachRequest(const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep);
  bool OnDetachRequest(const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep);
  bool HandleRequest(const gz::msgs::StringMsg_V &_req, bool _attach, gz::msgs::Boolean &_rep);

  /// Resolves a model+link name pair to a link Entity, gz::sim::kNullEntity if either isn't found.
  gz::sim::Entity FindLink(
      gz::sim::EntityComponentManager &_ecm, const std::string &_model, const std::string &_link);

  /// Runs on the simulation thread only (called from PreUpdate). Creates a components::DetachableJoint
  /// entity between the two resolved links -- reuses the existing joint (mirroring the original
  /// gazebo_ros_link_attacher plugin's getJoint/reuse behavior) if this exact model/link pair is
  /// already attached, rather than creating a duplicate.
  bool DoAttach(gz::sim::EntityComponentManager &_ecm, const JointKey &_key);

  /// Runs on the simulation thread only (called from PreUpdate). Removes the joint entity created
  /// by DoAttach, if any exists for this exact model/link pair.
  bool DoDetach(gz::sim::EntityComponentManager &_ecm, const JointKey &_key);

  gz::transport::Node node;

  std::mutex mutex;
  std::condition_variable cv;
  std::deque<std::shared_ptr<PendingRequest>> pending;

  /// Only ever read/written from PreUpdate (the simulation thread) -- no locking needed, unlike
  /// `pending`/`mutex` above which are shared with the gz-transport callback thread.
  std::map<JointKey, gz::sim::Entity> joints;
};
}  // namespace gz_link_attacher_sim

#endif  // GZ_LINK_ATTACHER_SIM_LINKATTACHER_HH_
