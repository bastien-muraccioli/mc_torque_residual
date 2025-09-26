#include "TorqueResidual.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

TorqueResidual::~TorqueResidual() = default;

void TorqueResidual::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  auto & robot = ctl.robot(ctl.robots()[0].name());
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  auto & rjo = robot.refJointOrder();

  dt_ = ctl.timestep();
  counter_ = 0.0;
  jointNumber = ctl.robot(ctl.robots()[0].name()).refJointOrder().size();

  // Make sure to have obstacle detection
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  ctl.controller().datastore().make<bool>("Torque Residual Obstacle detected", false);

  auto plugin_config = config("torque_residual");

  k_obs = plugin_config("k_obs", 10.0);
  threshold_filtering_ = plugin_config("threshold_filtering", 0.05);
  threshold_offset_ = plugin_config("threshold_offset");
  if(threshold_offset_.size() != jointNumber)
  {
    threshold_offset_ = Eigen::VectorXd::Constant(jointNumber, 10.0);
  }
  lpf_threshold_.setValues(threshold_offset_, threshold_filtering_, jointNumber);

  inertiaMatrix.resize(jointNumber, jointNumber);
  Eigen::VectorXd qdot(jointNumber);
  for(size_t i = 0; i < jointNumber; i++)
  {
    qdot[i] = robot.alpha()[robot.jointIndexByName(rjo[i])][0];
  }
  residual.setZero(jointNumber);
  residual_high_.setZero(jointNumber);
  residual_low_.setZero(jointNumber);
  integralTerm.setZero(jointNumber);

  coriolis = new rbd::Coriolis(robot.mb());
  forwardDynamics = rbd::ForwardDynamics(robot.mb());

  forwardDynamics.computeH(robot.mb(), robot.mbc());
  inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  pzero = inertiaMatrix * qdot;

  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::info("TorqueResidual::init called with configuration:\n{}", config.dump(true, true));
}

void TorqueResidual::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("TorqueResidual::reset called");
}

void TorqueResidual::before(mc_control::MCGlobalController & controller)
{ 
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  counter_ += dt_;

  if(activate_plot_ && !plot_added_)
  {
    addPlot(controller);
    plot_added_ = true;
  }

  // if(ctl.controller().datastore().has("Zurlo Collision Detection"))
  // {
  //   collision_stop_activated_zurlo_ = ctl.controller().datastore().get<bool>("Zurlo Collision Detection");
  // }

  residual_computation(controller);
  residual_high_ = lpf_threshold_.adaptiveThreshold(residual, true);
  residual_low_ = lpf_threshold_.adaptiveThreshold(residual, false);
  obstacle_detected_ = false;
  for (int i = 0; i < jointNumber; i++)
  {
    if (residual[i] > residual_high_[i] || residual[i] < residual_low_[i])
    {
      obstacle_detected_ = true;
      if(activate_verbose) mc_rtc::log::info("[Torque Residual] Obstacle detected on joint {}", i);
      if (collision_stop_activated_)
      {
        ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
      }
      break;
    }
  }

  ctl.controller().datastore().get<bool>("Torque Residual Obstacle detected") = obstacle_detected_;

  // mc_rtc::log::info("TorqueResidual::before");
}

void TorqueResidual::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("TorqueResidual::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration TorqueResidual::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void TorqueResidual::residual_computation(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  if(ctl.robot().encoderVelocities().empty())
  {
    mc_rtc::log::warning("[TorqueResidual] No encoder velocities available, skipping residual computation");
    return;
  }

  auto & robot = ctl.robot();
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  auto & rjo = realRobot.refJointOrder();

  Eigen::VectorXd qdot(jointNumber), tau(jointNumber);
  rbd::paramToVector(realRobot.alpha(), qdot);
  tau = Eigen::VectorXd::Map(realRobot.jointTorques().data(), realRobot.jointTorques().size());
  forwardDynamics.computeC(realRobot.mb(), realRobot.mbc());
  forwardDynamics.computeH(realRobot.mb(), realRobot.mbc());
  auto coriolisMatrix = coriolis->coriolis(realRobot.mb(), realRobot.mbc());
  auto coriolisGravityTerm = forwardDynamics.C();

  integralTerm += (tau + (coriolisMatrix + coriolisMatrix.transpose()) * qdot - coriolisGravityTerm + residual)
                  * ctl.timestep();
  inertiaMatrix = forwardDynamics.H() - forwardDynamics.HIr();
  auto pt = inertiaMatrix * qdot;

  residual = k_obs * (pt - integralTerm + pzero);
}

void TorqueResidual::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement({"Plugins", "TorqueResidual"},
    mc_rtc::gui::NumberInput("k_obs", [this]() { return k_obs; },
      [this](double k) 
      {
        this->integralTerm.setZero();
        this->residual.setZero();
        this->k_obs = k;
      }),
    mc_rtc::gui::IntegerInput("Residual shown", [this]() { return residual_shown_; },
      [this](int r) 
      {
        this->residual_shown_ = r;
      }),
    mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }),
    // Add checkbox to activate the collision stop
    mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
    mc_rtc::gui::Checkbox("Verbose", activate_verbose), 
    // Add Threshold offset input
    mc_rtc::gui::ArrayInput("Threshold offset", {"q_0", "q_1", "q_2", "q_3", "q_4", "q_5", "q_6"}, 
      [this](){return this->threshold_offset_;},
        [this](const Eigen::VectorXd & offset)
      { 
        threshold_offset_ = offset;
        lpf_threshold_.setOffset(threshold_offset_); 
      }),
    // Add Threshold filtering input
    mc_rtc::gui::NumberInput("Threshold filtering", [this](){return this->threshold_filtering_;},
        [this](double filtering)
      { 
        threshold_filtering_ = filtering;
        lpf_threshold_.setFiltering(threshold_filtering_); 
      })                                               
    );
}

void TorqueResidual::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("TorqueResidual_residual",
                                       [&, this]() { return this->residual; });
  ctl.controller().logger().addLogEntry("TorqueResidual_residual_high",
                                       [&, this]() { return this->residual_high_; });
  ctl.controller().logger().addLogEntry("TorqueResidual_residual_low",
                                       [&, this]() { return this->residual_low_; });
  ctl.controller().logger().addLogEntry("TorqueResidual_obstacleDetected",
                                       [&, this]() { return this->obstacle_detected_; });
}

void TorqueResidual::addPlot(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & gui = *ctl.controller().gui();

  gui.addPlot(
      "TorqueResidual",
      mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual_high_[residual_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual_low_[residual_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y("residual", [this]() { return residual[residual_shown_]; }, mc_rtc::gui::Color::Red)
    );
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TorqueResidual", mc_plugin::TorqueResidual)
