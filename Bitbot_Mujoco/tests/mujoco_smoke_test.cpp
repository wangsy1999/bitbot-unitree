#include "bitbot_mujoco/bus/mujoco_bus.h"
#include "bitbot_mujoco/device/mujoco_joint.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

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
  Require(model->nactuator == 29 && model->nu == 29, "Unexpected G1 actuator/control count");
  DataPtr data(mj_makeData(model.get()), mj_deleteData);
  Require(data != nullptr, "Cannot create robot data");
  pugi::xml_document config;
  Require(config.load_file(config_path), "Cannot load Bitbot configuration");
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

int main(int argc, char** argv) {
  try {
    Require(argc == 3, "Expected scene and Bitbot configuration paths");
    Require(mj_version() == mjVERSION_HEADER, "MuJoCo header/library version mismatch");
    CheckControlAddress();
    CheckRobot(argv[1], argv[2]);
    std::cout << "MuJoCo " << mj_versionString()
              << ": control addressing, device mappings and 500 physics steps passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
