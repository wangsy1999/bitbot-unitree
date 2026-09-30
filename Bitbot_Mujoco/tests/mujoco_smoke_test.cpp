#include "bitbot_mujoco/bus/mujoco_bus.h"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void CheckControlAddress() {
  // A multi-input actuator before a motor makes actuator ID != ctrl address.
  constexpr const char* xml = R"(
    <mujoco>
      <worldbody><body><joint name="servo"/><geom size="0.1"/>
        <body pos="0 0 0.3"><joint name="motor"/><geom size="0.1"/></body>
      </body></worldbody>
      <actuator><pid name="servo" joint="servo" kp="1" kv="1" input="pos vel"/>
        <motor name="motor" joint="motor"/></actuator>
    </mujoco>)";
  char error[1024] = {};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) throw std::runtime_error(error);
  ModelPtr model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  if (!model) throw std::runtime_error(mjs_getError(spec.get()));
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  Require(data != nullptr, "Cannot create fixture data");

  pugi::xml_document config;
  config.load_string(R"(<device id="0" type="MujocoJoint" name="motor"
    initial_pos="0" enable="true" mode="torque" pos_kp="0" pos_kd="0"
    pos_ki="0" vel_kp="0" vel_kd="0" vel_ki="0"/>)");
  bitbot::MujocoJoint joint(config.child("device"));
  bitbot::MujocoDevice& device = joint;
  device.UpdateModel(model.get(), data.get());
  joint.SetTargetTorque(0.25);
  device.Output(model.get(), data.get());
  Require(model->actuator_ctrladr[1] == 2, "Fixture does not exercise control addressing");
  Require(data->ctrl[0] == 0 && data->ctrl[1] == 0 && data->ctrl[2] == 0.25,
          "Motor command overwrote a different actuator's control input");
  mj_forward(model.get(), data.get());
  device.Input(model.get(), data.get());
  Require(std::abs(joint.GetActualTorque() - 0.25) < 1e-12,
          "Actuator force indexing changed");

  config.child("device").attribute("name").set_value("servo");
  bitbot::MujocoJoint unsupported(config.child("device"));
  bool rejected = false;
  try {
    static_cast<bitbot::MujocoDevice&>(unsupported).UpdateModel(model.get(), data.get());
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  Require(rejected, "Multi-input actuator must not be treated as a single-input motor");
}

void CheckRobot(const char* scene, const char* config_path) {
  char error[1024] = {};
  ModelPtr model(mj_loadXML(scene, nullptr, error, sizeof(error)), mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  Require(model->nactuator == 29 && model->nu == 87 && model->nout == 29, "Unexpected G1 actuator/control count");
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  Require(data != nullptr, "Cannot create robot data");
  pugi::xml_document config;
  Require(config.load_file(config_path), "Cannot load Bitbot configuration");
  int mit_count = 0;
  for (auto device : config.child("bitbot").child("bus").children("device")) {
    if (std::string(device.attribute("type").value()) == "MujocoJoint") {
      Require(std::string(device.attribute("mode").value()) == "mit",
              "G1 joint configuration must use MIT mode");
      ++mit_count;
    }
  }
  Require(mit_count == 29, "Unexpected G1 MIT joint count");
  bitbot::MujocoBus bus;
  bus.Configure(config.child("bitbot").child("bus"));
  bus.SetMujocoParam(model.get(), data.get());
  bus.UpdateDevices();
  mj_forward(model.get(), data.get());
  for (int step = 0; step < 500; ++step) {
    bus.ReadBus();
    bus.WriteBus();
    mj_step(model.get(), data.get());
    for (mjtSize i = 0; i < model->nq; ++i)
      Require(std::isfinite(data->qpos[i]), "Non-finite robot position");
    for (mjtSize i = 0; i < model->nv; ++i)
      Require(std::isfinite(data->qvel[i]), "Non-finite robot velocity");
    for (mjtSize i = 0; i < model->nsensordata; ++i)
      Require(std::isfinite(data->sensordata[i]), "Non-finite sensor value");
  }
  Require(std::abs(data->time - 1.0) < 1e-9, "Simulation time reset or timestep changed");
  Require(data->warning[mjWARN_BADQPOS].number == 0 &&
          data->warning[mjWARN_BADQVEL].number == 0 &&
          data->warning[mjWARN_BADQACC].number == 0, "Robot dynamics diverged");
}

void CheckNativePid() {
  constexpr const char* xml = R"(
    <mujoco>
      <worldbody><body><joint name="before"/><geom size="0.1"/>
        <body pos="0 0 0.3"><joint name="servo"/><geom size="0.1"/>
          <body pos="0 0 0.3"><joint name="after"/><geom size="0.1"/></body>
        </body>
      </body></worldbody>
      <actuator>
        <motor name="before" joint="before"/>
        <pid name="servo" joint="servo" input="pos vel ff" kp="10" kv="2"
          ki="0" forcerange="-5 5"/>
        <motor name="after" joint="after"/>
      </actuator>
    </mujoco>)";
  char error[1024] = {};
  std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
      mj_parseXMLString(xml, nullptr, error, sizeof(error)), mj_deleteSpec);
  if (!spec) throw std::runtime_error(error);
  ModelPtr model(mj_compile(spec.get(), nullptr), mj_deleteModel);
  if (!model) throw std::runtime_error(mjs_getError(spec.get()));
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  Require(data != nullptr, "Cannot create PID fixture data");
  Require(model->nu == 5 && model->actuator_ctrladr[1] == 1 &&
          model->actuator_ctrladr[2] == 4, "Unexpected PID fixture control layout");

  pugi::xml_document config;
  config.load_string(R"(<device id="0" type="MujocoJoint" name="servo"
    initial_pos="0" enable="true" mode="mit" pos_kp="10" pos_kd="2"
    pos_ki="0" vel_kp="0" vel_kd="0" vel_ki="0"/>)");
  bitbot::MujocoJoint joint(config.child("device"));
  bitbot::MujocoDevice& device = joint;
  device.UpdateModel(model.get(), data.get());
  Require(bitbot::MujocoJoint::ModeFromStr("mit") == bitbot::MujocoJointMode::MIT &&
          bitbot::MujocoJoint::ModeToStr(bitbot::MujocoJointMode::MIT) == "mit",
          "MIT mode configuration does not round-trip");
  data->ctrl[0] = 0.25;
  data->ctrl[4] = 0.75;
  data->qpos[model->jnt_qposadr[1]] = 0.3;
  data->qvel[model->jnt_dofadr[1]] = -0.2;
  joint.SetTargetPosition(0.5);
  joint.SetTargetVelocity(0.4);
  joint.SetTargetTorque(1.5);
  joint.SetMotionKi(999); // The MIT adapter does not enable PID integral action.
  for (int cycle = 0; cycle < 3; ++cycle) {
    device.Output(model.get(), data.get());
    Require(data->ctrl[1] == 0.5 && data->ctrl[2] == 0.4 && data->ctrl[3] == 1.5,
            "Native PID must receive position, velocity and feedforward inputs");
    mj_forward(model.get(), data.get());
    device.Input(model.get(), data.get());
    Require(std::abs(joint.GetActualTorque() - 4.7) < 1e-12,
            "MuJoCo native PID produced the wrong MIT force");
  }
  Require(data->ctrl[0] == 0.25 && data->ctrl[4] == 0.75,
          "PID command overwrote a neighboring motor input");

  // Change physical state without refreshing the device's cached observations:
  // the native controller must use MuJoCo state, not a precomputed C++ torque.
  data->qpos[model->jnt_qposadr[1]] = 0.35;
  data->qvel[model->jnt_dofadr[1]] = 0.1;
  mj_forward(model.get(), data.get());
  Require(std::abs(data->actuator_force[1] - 3.6) < 1e-12,
          "Native PID failed to recompute force from the current physical state");
  data->qpos[model->jnt_qposadr[1]] = 0.3;
  data->qvel[model->jnt_dofadr[1]] = -0.2;
  joint.SetMotionKp(4);
  joint.SetMotionKd(3);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(std::abs(data->actuator_force[1] - 4.1) < 1e-12,
          "Native PID ignored runtime gain changes");
  joint.SetMotionKp(0);
  joint.SetMotionKd(0);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->actuator_force[1] == 1.5, "Zero MIT gains must retain feedforward torque");
  joint.SetMotionKd(3);
  joint.SetTargetVelocity(0);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(std::abs(data->actuator_force[1] - 2.1) < 1e-12,
          "Zero stiffness must retain MIT damping and feedforward");
  joint.SetMotionKp(10);
  joint.SetMotionKd(2);
  joint.SetTargetTorque(0);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(std::abs(data->actuator_force[1] - 2.4) < 1e-12,
          "Native MIT with zero velocity/feedforward must match legacy PD");
  joint.SetTargetTorque(100);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->ctrl[3] == 100 && data->actuator_force[1] == 5,
          "Native PID must limit total force, not clamp setpoint inputs");
  joint.SetTargetTorque(-100);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->actuator_force[1] == -5, "Native PID negative force limit failed");
  joint.SetMode(bitbot::MujocoJointMode::NONE);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->actuator_force[1] == 0, "None mode must disable native PID feedback");
  joint.SetMode(bitbot::MujocoJointMode::MIT);
  joint.SetTargetVelocity(0.4);
  joint.SetTargetTorque(1.5);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(std::abs(data->actuator_force[1] - 4.7) < 1e-12,
          "Native PID failed to restore gains after none mode");
  joint.SetMode(bitbot::MujocoJointMode::TORQUE);
  device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->actuator_force[1] == 1.5, "Torque mode must disable native PID feedback");

  config.child("device").attribute("enable").set_value(false);
  bitbot::MujocoJoint disabled(config.child("device"));
  auto& disabled_device = static_cast<bitbot::MujocoDevice&>(disabled);
  disabled_device.UpdateModel(model.get(), data.get());
  disabled.SetTargetTorque(3);
  disabled.SetTargetPosition(1);
  disabled_device.Output(model.get(), data.get());
  mj_forward(model.get(), data.get());
  Require(data->ctrl[1] == 0 && data->ctrl[2] == 0 && data->ctrl[3] == 0 &&
          data->actuator_force[1] == 0, "Disabled PID must clear commands and feedback");

  config.child("device").attribute("name").set_value("before");
  bitbot::MujocoJoint wrong_motor(config.child("device"));
  bool rejected = false;
  try {
    static_cast<bitbot::MujocoDevice&>(wrong_motor).UpdateModel(model.get(), data.get());
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  Require(rejected, "MIT mode must require a native PID actuator");
}

int main(int argc, char** argv) {
  try {
    Require(argc == 3, "Expected scene and Bitbot configuration paths");
    Require(mj_version() == mjVERSION_HEADER, "MuJoCo header/library version mismatch");
    CheckControlAddress();
    CheckNativePid();
    CheckRobot(argv[1], argv[2]);
    std::cout << "MuJoCo " << mj_versionString()
              << ": control addressing, MIT commands, device mappings and 500 physics steps passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
