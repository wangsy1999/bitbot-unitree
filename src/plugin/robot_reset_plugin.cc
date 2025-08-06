#include <gz/msgs/int32.pb.h>

#include <gz/common/KeyEvent.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class RobotResetPlugin : public System,
                         public ISystemConfigure,
                         public ISystemPreUpdate {
 public:
  RobotResetPlugin() = default;
  ~RobotResetPlugin() = default;

  virtual void Configure(const Entity& entity,
                         const std::shared_ptr<const sdf::Element>& sdf,
                         EntityComponentManager& ecm,
                         EventManager& event_mgr) override {
    robot_name_ = sdf->Get<std::string>("robot_name");
    reset_height_ = sdf->Get<double>("reset_height");

    this->node_.Subscribe("/keyboard/keypress", &RobotResetPlugin::OnKeyPress,
                          this);
  }

  virtual void PreUpdate(const UpdateInfo& info,
                         EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }

    // Find model entity
    if (model_entity_ == gz::sim::kNullEntity) {
      ecm.Each<components::Name, components::Model>(
          [&](const Entity& entity, const components::Name* name,
              const components::Model* model) -> bool {
            if (name->Data() == robot_name_) {
              model_entity_ = entity;
              return false;
            }
            return true;
          });
    }

    // Find joint entities
    if (joint_entities_.size() == 0) {
      ecm.Each<components::Name, gz::sim::components::ParentEntity,
               components::Joint>(
          [&](const Entity& entity, const components::Name* name,
              const components::ParentEntity* parent_entity,
              const components::Joint* joint) -> bool {
            if (parent_entity->Data() == model_entity_) {
              joint_entities_.push_back(entity);
            }
            return true;
          });
    }

    // Find link entities
    if (link_entities_.size() == 0) {
      ecm.Each<components::Name, components::ParentEntity, components::Link>(
          [&](const Entity& entity, const components::Name* name,
              components::ParentEntity* parent_entity,
              components::Link* link) -> bool {
            link_entities_.push_back(entity);
            return true;
          });
    }

    if (cmd_clean_flag_.load()) {
      for (auto& link_entity : link_entities_) {
        // This is a bug of gazebo harmonic. Velocity commands are not removed
        // automatically
        gz::sim::Link link(link_entity);
        ecm.RemoveComponent<components::LinearVelocityCmd>(link_entity);
        ecm.RemoveComponent<components::AngularVelocityCmd>(link_entity);
      }
      cmd_clean_flag_.store(false);
    }

    if (reset_requested_.load()) {
      // Set joint zero position
      for (auto& joint_entity : joint_entities_) {
        gz::sim::Joint joint(joint_entity);
        joint.ResetPosition(ecm, {0.0});
        joint.SetVelocity(ecm, {0.0});
      }
      // Set link velocity
      for (auto& link_entity : link_entities_) {
        gz::sim::Link link(link_entity);
        ecm.CreateComponent(link_entity,
                            components::LinearVelocityCmd({0.0, 0.0, 0.0}));
        ecm.CreateComponent(link_entity,
                            components::AngularVelocityCmd({0.0, 0.0, 0.0}));
      }
      // Reset base pose
      gz::sim::Model model(model_entity_);
      model.SetWorldPoseCmd(ecm, math::Pose3d(0, 0, reset_height_, 0, 0, 0));
      cmd_clean_flag_.store(true);
      reset_requested_.store(false);
    }
  }

 private:
  void OnKeyPress(const msgs::Int32& event) {
    // Backspace
    if (event.data() == 16777219) {
      reset_requested_.store(true);
      gzmsg << "Reset requested for robot: " << robot_name_ << std::endl;
    }
  }

 private:
  std::string robot_name_;
  double reset_height_;
  std::list<gz::sim::Entity> joint_entities_;
  std::list<gz::sim::Entity> link_entities_;
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};

  transport::Node node_;
  std::atomic_bool reset_requested_{false};
  std::atomic_bool cmd_clean_flag_{false};
};

GZ_ADD_PLUGIN(RobotResetPlugin, gz::sim::System,
              RobotResetPlugin::ISystemConfigure,
              RobotResetPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(RobotResetPlugin, "gz::sim::systems::RobotResetPlugin")
