#include "LinkAttacher.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

using namespace gz_link_attacher_sim;

namespace
{
constexpr char kAttachTopic[] = "/link_attacher/attach";
constexpr char kDetachTopic[] = "/link_attacher/detach";
}  // namespace

void LinkAttacher::Configure(
    const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &)
{
  this->node.Advertise(kAttachTopic, &LinkAttacher::OnAttachRequest, this);
  this->node.Advertise(kDetachTopic, &LinkAttacher::OnDetachRequest, this);
}

bool LinkAttacher::OnAttachRequest(const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep)
{
  return this->HandleRequest(_req, true, _rep);
}

bool LinkAttacher::OnDetachRequest(const gz::msgs::StringMsg_V &_req, gz::msgs::Boolean &_rep)
{
  return this->HandleRequest(_req, false, _rep);
}

bool LinkAttacher::HandleRequest(const gz::msgs::StringMsg_V &_req, bool _attach, gz::msgs::Boolean &_rep)
{
  // Malformed request (wrong arity) is a service-call-level failure, not a "no such link" one --
  // reject before ever queuing, rather than waiting a full simulation step to say no.
  if (_req.data_size() != 4)
  {
    _rep.set_data(false);
    return true;
  }

  auto request = std::make_shared<PendingRequest>();
  request->key = JointKey(_req.data(0), _req.data(1), _req.data(2), _req.data(3));
  request->attach = _attach;

  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending.push_back(request);
  }

  // Blocks this gz-transport callback thread until PreUpdate (the simulation thread) has actually
  // done the ECM work and populated the result -- see PendingRequest's doc comment for why this
  // can't just be fire-and-forget: the service contract requires _rep to be fully populated before
  // this function returns.
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    this->cv.wait(lock, [&request]() { return request->done; });
  }
  _rep.set_data(request->result);
  return true;
}

void LinkAttacher::PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm)
{
  std::deque<std::shared_ptr<PendingRequest>> toProcess;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    std::swap(toProcess, this->pending);
  }
  if (toProcess.empty())
  {
    return;
  }

  for (auto &request : toProcess)
  {
    request->result =
        request->attach ? this->DoAttach(_ecm, request->key) : this->DoDetach(_ecm, request->key);
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      request->done = true;
    }
  }
  // One notify_all after the whole batch, not per-request inside the loop above -- avoids waking
  // every other still-waiting caller on each individual completion when several attach/detach
  // calls land in the same simulation step.
  this->cv.notify_all();
}

gz::sim::Entity LinkAttacher::FindLink(
    gz::sim::EntityComponentManager &_ecm, const std::string &_model, const std::string &_link)
{
  auto modelEntity = _ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(_model));
  if (modelEntity == gz::sim::kNullEntity)
  {
    return gz::sim::kNullEntity;
  }
  return gz::sim::Model(modelEntity).LinkByName(_ecm, _link);
}

bool LinkAttacher::DoAttach(gz::sim::EntityComponentManager &_ecm, const JointKey &_key)
{
  auto existing = this->joints.find(_key);
  if (existing != this->joints.end() && _ecm.HasEntity(existing->second))
  {
    // Already attached -- mirrors the original gazebo_ros_link_attacher plugin's getJoint/reuse
    // behavior (a repeat attach call for the same pair is a success, not an error).
    return true;
  }

  auto link1 = this->FindLink(_ecm, std::get<0>(_key), std::get<1>(_key));
  auto link2 = this->FindLink(_ecm, std::get<2>(_key), std::get<3>(_key));
  if (link1 == gz::sim::kNullEntity || link2 == gz::sim::kNullEntity)
  {
    return false;
  }

  // The entire runtime joint-creation mechanism -- confirmed directly against gz-sim8's own
  // DetachableJoint system source, not assumed: creating an entity with just this one component
  // is everything the Physics system needs to instantiate a real fixed joint between the two
  // links. No Name/Pose/ParentEntity companion components required.
  auto jointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(
      jointEntity, gz::sim::components::DetachableJoint({link1, link2, "fixed"}));
  this->joints[_key] = jointEntity;
  return true;
}

bool LinkAttacher::DoDetach(gz::sim::EntityComponentManager &_ecm, const JointKey &_key)
{
  auto existing = this->joints.find(_key);
  if (existing == this->joints.end())
  {
    return false;
  }
  // Queues removal rather than erasing the entity outright -- lets the physics engine process the
  // detachment first, same reasoning gz-sim's own DetachableJoint system uses for this same call.
  _ecm.RequestRemoveEntity(existing->second);
  this->joints.erase(existing);
  return true;
}

GZ_ADD_PLUGIN(LinkAttacher, gz::sim::System, LinkAttacher::ISystemConfigure, LinkAttacher::ISystemPreUpdate)
