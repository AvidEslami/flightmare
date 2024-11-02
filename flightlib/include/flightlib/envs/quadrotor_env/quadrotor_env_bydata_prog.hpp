#pragma once

// std lib
#include <stdlib.h>
#include <cmath>
#include <iostream>

// yaml cpp
#include <yaml-cpp/yaml.h>

// flightlib

#include "quadenv_ctl.hpp"


namespace flightlib {

// namespace quadenv {

// enum Ctl : int {
//   // observations
//   kObs = 0,
//   //
//   kPos = 0,
//   kNPos = 3,
//   kOri = 3,
//   kNOri = 3,
//   kLinVel = 6,
//   kNLinVel = 3,
//   kAngVel = 9,
//   kNAngVel = 3,
//   kNObs = 12,
//   // control actions
//   kAct = 0,
//   kNAct = 4,
// };
// };

class QuadrotorEnvByDataProg final : public EnvBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  QuadrotorEnvByDataProg();
  QuadrotorEnvByDataProg(const std::string &cfg_path);
  ~QuadrotorEnvByDataProg();

  // - public OpenAI-gym-style functions
  bool reset(Ref<Vector<>> obs, const bool random = true) override;
  bool resetRange(Ref<Vector<>> obs, int lower_zbound, int upper_zbound, int lower_xybound, int upper_xybound, const bool random = true) override;
  Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) override;

  // - public set functions
  bool loadParam(const YAML::Node &cfg);

  // - public get functions
  bool getObs(Ref<Vector<>> obs) override;
  bool getAct(Ref<Vector<>> act) const;
  bool getAct(Command *const cmd) const;

  // - auxiliar functions
  bool isTerminalState(Scalar &reward) override;
  void addObjectsToUnity(std::shared_ptr<UnityBridge> bridge);

  friend std::ostream &operator<<(std::ostream &os,
                                  const QuadrotorEnvByDataProg &quad_env);

 private:
  // quadrotor
  std::shared_ptr<Quadrotor> quadrotor_ptr_;
  QuadState quad_state_;
  Command cmd_;
  Logger logger_{"QuadrotorEnvByDataProg"};

  // Define reward for training
  Scalar pos_coeff_, ori_coeff_, lin_vel_coeff_, ang_vel_coeff_, act_coeff_;

  // observations and actions (for RL)
  Vector<quadenv::kNObs> quad_obs_;
  Vector<quadenv::kNAct> quad_act_;
  // Store a trajectory as a list of states (pos, ori, lin_vel -> 10 elements)
  std::vector<Vector<10>> traj_;
  int mid_train_step_;
  

  // reward function design (for model-free reinforcement learning)
  Vector<quadenv::kNObs> goal_state_;

  // action and observation normalization (for learning)
  Vector<quadenv::kNAct> act_mean_;
  Vector<quadenv::kNAct> act_std_;
  Vector<quadenv::kNObs> obs_mean_ = Vector<quadenv::kNObs>::Zero();
  Vector<quadenv::kNObs> obs_std_ = Vector<quadenv::kNObs>::Ones();

  YAML::Node cfg_;
  Matrix<3, 2> world_box_;
};

}  // namespace flightlib