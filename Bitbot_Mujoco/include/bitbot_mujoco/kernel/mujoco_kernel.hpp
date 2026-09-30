#pragma once

#include "bitbot_kernel/kernel/kernel.hpp"
#include "bitbot_mujoco/bus/mujoco_bus.h"
#include "bitbot_mujoco/kernel/mujoco_utils.h"

#include "mujoco/mujoco.h"
#include "simulate.h"

#include <thread>
#include <stdexcept>

namespace bitbot
{
  // 将类成员函数转换为函数指针
  template <typename T, int tag = 0>
  struct Callback;
  template <typename Ret, typename... Params, int tag>
  struct Callback<Ret(Params...), tag> {
    template <typename... Args>
    static Ret callback(Args... args) {
        return func(args...);
    }
    int m_tag = tag;
    static std::function<Ret(Params...)> func;
  };
  template <typename Ret, typename... Params, int tag>
  std::function<Ret(Params...)> Callback<Ret(Params...), tag>::func;

  template <typename UserData>
  class MujocoKernel : public KernelTpl<MujocoKernel<UserData>, MujocoBus, UserData>
  {
  public:
    MujocoKernel(std::string config_file)
      : KernelTpl<MujocoKernel<UserData>, MujocoBus, UserData>(config_file)
    {
      MujocoKernelInit();
    }
    ~MujocoKernel() = default;

  protected:
    void doStart()
    {
      sim_->run = 1;
    }

    void doRun()
    {
      // start physics thread
      std::thread physicsthreadhandle = std::thread(&MujocoKernel::PhysicsThread, this);

      // start simulation UI loop (blocking call)
      sim_->RenderLoop();
      physicsthreadhandle.join();

    };

    void MujocoConfigure(const pugi::xml_node& bitbot_node)
    {
      using namespace std::filesystem;

      auto mujoco_node = bitbot_node.child("mujoco");
      if(mujoco_node == NULL)
      {
        this->logger_->error("Config file need node: mujoco");
        return;
      }

      mujoco_file_ = mujoco_node.attribute("file").as_string();
      if(mujoco_file_.empty())
      {
        this->logger_->error("mujoco node does not have attribute \'file\'.");
      }
      else
      {
        path mujoco_file_path(mujoco_file_);
        if(mujoco_file_path.is_relative())
        {
          mujoco_file_path = canonical(this->parser_->FilePath().parent_path() / mujoco_file_path);
          mujoco_file_ = mujoco_file_path.string();
        }
      }
      this->logger_->debug("MuJoCo file: {}", mujoco_file_);

      // request loadmodel if file given (otherwise drag-and-drop)
      if (!mujoco_file_.empty()) {
        mj_m_ = LoadModel(mujoco_file_.c_str(), *sim_);
        if (mj_m_) mj_d_ = mj_makeData(mj_m_);
        if (mj_d_) {
          this->logger_->debug("mass: {}, CoM: {},{},{}", mj_m_->body_subtreemass[0], mj_d_->subtree_com[0], mj_d_->subtree_com[1], mj_d_->subtree_com[2]);
          this->logger_->debug("nq: {} nv: {} nu: {} nbody: {} njnt: {}", mj_m_->nq, mj_m_->nv, mj_m_->nu, mj_m_->nbody, mj_m_->njnt);
          this->busmanager_.SetMujocoParam(mj_m_, mj_d_);
        }
      }
    }

    void PowerOn(int, KernelInterface&)
    {
      this->kernel_config_data_.power_on_finish_flag=true;
    }

  private:
    void MujocoKernelInit()
    {
      namespace mj = ::mujoco;

      // print version, check compatibility
      this->logger_->info("MuJoCo version {}", mj_versionString());
      if (mjVERSION_HEADER!=mj_version()) {
        throw std::runtime_error("MuJoCo headers and library have different versions");
      }

      // RegisterCmdCallback("power_on", std::bind(&MujocoKernel::PowerOn, this));

      // scan for libraries in the plugin directory to load additional plugins
      scanPluginLibraries();

      static mjvCamera cam;
      mjv_defaultCamera(&cam);

      static mjvOption opt;
      mjv_defaultOption(&opt);

      static mjvPerturb pert;
      mjv_defaultPerturb(&pert);

      // simulate object encapsulates the UI
      sim_ = std::make_unique<mj::Simulate>(
          std::make_unique<mj::GlfwAdapter>(),
          &cam, &opt, &pert, /* is_passive = */ false
      );
      sim_->run = 0;

      MujocoConfigure(this->parser_->GetBitbotNode());
    }

    void PhysicsThread()
    {
      if(mj_m_ && mj_d_)
      {
        sim_->LoadMessage(mujoco_file_.c_str());
        sim_->Load(mj_m_, mj_d_, mujoco_file_.c_str());
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        this->busmanager_.UpdateDevices();
        mj_forward(mj_m_, mj_d_);
      }

      Callback<void(const mjModel*, mjData*)>::func = std::bind(&MujocoKernel::mjcbControl, this, std::placeholders::_1, std::placeholders::_2);
      mjcb_control = static_cast<mjfGeneric>(Callback<void(const mjModel*, mjData*)>::callback);

      PhysicsLoop();

      // delete everything we allocated
      mjcb_control = nullptr;
      mj_deleteData(mj_d_);
      mj_deleteModel(mj_m_);
      mj_d_ = nullptr;
      mj_m_ = nullptr;
      this->busmanager_.SetMujocoParam(nullptr, nullptr);
    }

    void PhysicsLoop()
    {
      namespace mj = ::mujoco;
      using Seconds = std::chrono::duration<double>;

      // cpu-sim syncronization point
      std::chrono::time_point<mj::Simulate::Clock> syncCPU;
      mjtNum syncSim = 0;

      // run until asked to exit
      while (!sim_->exitrequest.load()) {
        if (sim_->droploadrequest.exchange(0)) {
          ReloadModel(sim_->dropfilename);
        }
        if (sim_->uiloadrequest.load()) {
          sim_->uiloadrequest.fetch_sub(1);
          ReloadModel(sim_->filename);
        }

        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if (sim_->run && sim_->busywait) {
          std::this_thread::yield();
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        {
          // lock the sim mutex
          const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

          // run only if model is present
          if (mj_m_) {
            // running
            if (sim_->run && !this->kernel_config_data_.stop_flag /*&& this->kernel_config_data_.power_on_finish_flag*/) {
            // if (sim_->run && !this->kernel_config_data_.stop_flag && this->kernel_config_data_.power_on_finish_flag) {
              // record cpu time at start of iteration
              const auto startCPU = mj::Simulate::Clock::now();

              // elapsed CPU and simulation time since last sync
              const auto elapsedCPU = startCPU - syncCPU;
              double elapsedSim = mj_d_->time - syncSim;

              // requested slow-down factor
              double slowdown = 100 / sim_->percentRealTime[sim_->real_time_index];

              // misalignment condition: distance from target sim time is bigger than syncmisalign
              bool misaligned = mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

              // out-of-sync (for any reason): reset sync times, step
              if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                  misaligned || sim_->speed_changed) {                // re-sync
                syncCPU = startCPU;
                syncSim = mj_d_->time;
                sim_->speed_changed = false;

                // run single step, let next iteration deal with timing
                sim_->InjectNoise(sim_->key);
                mj_step(mj_m_, mj_d_);

                this->KernelPrivateLoopEndTask();
                sim_->AddToHistory();
              }

              // in-sync: step until ahead of cpu
              else {
                bool measured = false;
                mjtNum prevSim = mj_d_->time;
                double refreshTime = simRefreshFraction/sim_->refresh_rate;

                // step while sim lags behind cpu and within refreshTime
                while (Seconds((mj_d_->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
                  // measure slowdown before first step
                  if (!measured && elapsedSim) {
                    sim_->measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;;
                    measured = true;
                  }

                  // call mj_step
                  sim_->InjectNoise(sim_->key);
                  mj_step(mj_m_, mj_d_);

                  this->KernelPrivateLoopEndTask();
                  sim_->AddToHistory();

                  // break if reset
                  if (mj_d_->time < prevSim) {
                    break;
                  }
                }
              }
            }

            // paused
            else {
              // run mj_forward, to update rendering and joint sliders
              mj_forward(mj_m_, mj_d_);
              sim_->speed_changed = true;

              this->busmanager_.ReadBus();
              this->UpdateRuntimeData();
              this->backend_->SetMonitorData(this->monitor_data_);

              this->HandleEvents();
            }
          }
        }  // release std::lock_guard<std::mutex>
      }

    }

    void ReloadModel(const char* filename)
    {
      // Load waits for the main/render thread; do not hold its mutex here.
      const std::string requested_file(filename);
      sim_->LoadMessage(requested_file.c_str());
      mjModel* new_model = LoadModel(requested_file.c_str(), *sim_);
      mjData* new_data = new_model ? mj_makeData(new_model) : nullptr;
      if (!new_data) {
        mj_deleteModel(new_model);
        sim_->LoadMessageClear();
        return;
      }
      sim_->Load(new_model, new_data, requested_file.c_str());
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      mj_deleteData(mj_d_);
      mj_deleteModel(mj_m_);
      mj_m_ = new_model;
      mj_d_ = new_data;
      mujoco_file_ = requested_file;
      control_step_initialized_ = false;
      this->busmanager_.SetMujocoParam(mj_m_, mj_d_);
      this->busmanager_.UpdateDevices();
      mj_forward(mj_m_, mj_d_);
      sim_->speed_changed = true;
    }

    void mjcbControl(const mjModel* m, mjData* d)
    {
      using namespace std::chrono;
      const auto this_step = static_cast<uint64_t>(d->time / m->opt.timestep);
      const double this_mjtime = d->time * 1e3;
      // UI reset/history can rewind simulation time. Keep timing per kernel and
      // rebase it instead of waiting for the previous run's step count.
      if (!control_step_initialized_ || this_mjtime < last_mjtime_) {
        last_step_ = this_step;
        last_mjtime_ = this_mjtime;
        control_step_initialized_ = true;
        return;
      }

      if(this_step > last_step_)
      {
        auto start_time = std::chrono::high_resolution_clock::now();

        this->kernel_runtime_data_.periods_count++;
        this->kernel_runtime_data_.period = this_mjtime - last_mjtime_;

        // this->logger_->debug("torque: act {} qfrc {}", d->actuator_force[0], d->qfrc_actuator[6]);

        this->HandleEvents();
        this->KernelLoopTask();

        last_mjtime_ = this_mjtime;

        auto end_time = std::chrono::high_resolution_clock::now();
        this->kernel_runtime_data_.process_time = duration_cast<nanoseconds>(end_time - start_time).count()/1e6;
      }

      last_step_ = this_step;
    }

    std::string mujoco_file_;

    std::unique_ptr<mujoco::Simulate> sim_ = nullptr;
    mjModel* mj_m_ = nullptr;
    mjData* mj_d_ = nullptr;
    bool control_step_initialized_ = false;
    uint64_t last_step_ = 0;
    double last_mjtime_ = 0;

    double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
    double simRefreshFraction = 0.7;  // fraction of refresh available for simulation

    std::unique_ptr<std::thread> physics_thread_;
  };


}
