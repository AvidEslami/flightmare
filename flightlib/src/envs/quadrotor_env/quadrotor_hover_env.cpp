#include "flightlib/envs/quadrotor_env/quadrotor_hover_env.hpp"

namespace flightlib {

int trial_num = 0;

QuadrotorHoverEnv::QuadrotorHoverEnv()
  : QuadrotorHoverEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

QuadrotorHoverEnv::QuadrotorHoverEnv(const std::string &cfg_path)
  : EnvBase(),
    pos_coeff_(0.0),
    ori_coeff_(0.0),
    lin_vel_coeff_(0.0),
    ang_vel_coeff_(0.0),
    act_coeff_(0.0),
    goal_state_((Vector<quadenv::kNObs>() << 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0)
                  .finished()) {
  // load configuration file
  YAML::Node cfg_ = YAML::LoadFile(cfg_path);

  std::random_device rd;
  random_gen_ = std::mt19937(rd());

  quadrotor_ptr_ = std::make_shared<Quadrotor>();
  // update dynamics
  QuadrotorDynamics dynamics;
  dynamics.updateParams(cfg_);
  quadrotor_ptr_->updateDynamics(dynamics);

  // define a bounding box
  world_box_ << -20, 20, -20, 20, 0, 20;
  if (!quadrotor_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  // obs_dim_ = quadenv::kNObs * 2;

  // NEW APPROACH: Let us just focus on relative positions
  obs_dim_ = quadenv::kNObs;


  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);
}

QuadrotorHoverEnv::~QuadrotorHoverEnv() {}

bool QuadrotorHoverEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  // quad_state_.x(QS::POSX) = 0.0;
  // quad_state_.x(QS::POSY) = 0.0;
  // quad_state_.x(QS::POSZ) = 3.0;
  // quad_state_.x(QS::VELX) = 0.0;
  // quad_state_.x(QS::VELY) = 0.0;
  // quad_state_.x(QS::VELZ) = 0.0;
  // quad_state_.x(QS::ATTW) = 0.0;
  // quad_state_.x(QS::ATTX) = 0.0;
  // quad_state_.x(QS::ATTY) = 0.0;
  // quad_state_.x(QS::ATTZ) = 0.0;
  // // quad_state_.qx /= quad_state_.qx.norm();

  // if (trial_num == 0) {
  //   goal_state_(QS::POSX) = 3.0;
  // }
  // else if (trial_num == 1) {
  //   goal_state_(QS::POSX) = 3.0;
  // }
  // else if (trial_num == 2) {
  //   goal_state_(QS::POSX) = 3.0;
  // }
  // else {
  //   goal_state_(QS::POSX) = 10.0;
  // }
  // goal_state_(QS::POSY) = 0.0;
  // goal_state_(QS::POSZ) = 3.0;
  if (random) {

    quad_state_.setZero();
    quad_state_.x(QS::POSZ) = 5.0;

    if (trial_num == 0) {
      goal_state_(QS::POSX) = 3.0;
    }
    else if (trial_num == 1) {
      goal_state_(QS::POSX) = 6.0;
    }
    else if (trial_num == 2) {
      goal_state_(QS::POSX) = 15.0;
    }
    else {
      goal_state_(QS::POSX) = 0.0;
    }
    // goal_state_(QS::POSX) = 2.0;
    goal_state_(QS::POSY) = 0.0;
    goal_state_(QS::POSZ) = 5;
  }

  trial_num += 1;
  std::cout << "trial_num: " << trial_num << std::endl;
  // Print Starting Position and Goal Position
  std::cout << "Starting Position: " << quad_state_.x(QS::POSX) << ", " << quad_state_.x(QS::POSY) << ", " << quad_state_.x(QS::POSZ) << std::endl;
  std::cout << "Goal Position: " << goal_state_(QS::POSX) << ", " << goal_state_(QS::POSY) << ", " << goal_state_(QS::POSZ) << std::endl;
  // reset quadrotor with random states
  quadrotor_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.thrusts.setZero();

  // obtain observations
  getObs(obs);
  return true;
}


bool QuadrotorHoverEnv::getObs(Ref<Vector<>> obs) {
  quadrotor_ptr_->getState(&quad_state_);

  // convert quaternion to euler angle
  Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
  // quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w;

  // obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
  //   obs.segment<quadenv::kNObs>(quadenv::kObs + quadenv::kNObs) = goal_state_;
  // obs(quadenv::kNObs) = goal_state_(QS::POSZ); // add goal state to observation vector


  // NEW APPROACH: Let us just focus on relative positions
  //               This lets us reduce model size and improve performance/generalization speed
  obs.segment<quadenv::kNObs>(quadenv::kObs) = goal_state_.segment<quadenv::kNObs>(quadenv::kObs) - quad_obs_.segment<quadenv::kNObs>(quadenv::kObs);

  return true;
}

Scalar QuadrotorHoverEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  cmd_.t += sim_dt_;
  cmd_.thrusts = quad_act_;

  // simulate quadrotor
  quadrotor_ptr_->run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  Matrix<3, 3> rot = quad_state_.q().toRotationMatrix();

  // ---------------------- reward function design
  // - position tracking
  Scalar pos_reward =
    pos_coeff_ * (quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) -
                  goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
                   .squaredNorm()*2;

  // - orientation tracking
  Scalar ori_reward =
    ori_coeff_ * (quad_obs_.segment<quadenv::kNOri>(quadenv::kOri) -
                  goal_state_.segment<quadenv::kNOri>(quadenv::kOri))
                   .squaredNorm();
  // - linear velocity tracking
  Scalar lin_vel_reward =
    lin_vel_coeff_ * (quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel) -
                      goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel))
                       .squaredNorm();
  // - angular velocity tracking
  Scalar ang_vel_reward =
    ang_vel_coeff_ * (quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel) -
                      goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel))
                       .squaredNorm();

  // - control action penalty
  Scalar act_reward = act_coeff_ * act.cast<Scalar>().norm();

  Scalar total_reward =
    pos_reward + ori_reward + lin_vel_reward + ang_vel_reward + act_reward;

  // survival reward
  total_reward += 0.1;

  return total_reward;
}

bool QuadrotorHoverEnv::isTerminalState(Scalar &reward) {
  if (quad_state_.x(QS::POSZ) <= 0.02) {
    reward = -0.02;
    return true;
  }
  // We want the quadrotor to terminate within 0.1m of the goal, and reward it immediately for doing so
  // if ((quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) -
  //      goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
  //       .squaredNorm() < 0.1) {
  //   reward = 10.0;
  //   return true;
  // }
  reward = 0.0;
  return false;
}

bool QuadrotorHoverEnv::loadParam(const YAML::Node &cfg) {
  if (cfg["quadrotor_env"]) {
    sim_dt_ = cfg["quadrotor_env"]["sim_dt"].as<Scalar>();
    max_t_ = cfg["quadrotor_env"]["max_t"].as<Scalar>();
  } else {
    return false;
  }

  if (cfg["rl"]) {
    // load reinforcement learning related parameters
    pos_coeff_ = cfg["rl"]["pos_coeff"].as<Scalar>();
    ori_coeff_ = cfg["rl"]["ori_coeff"].as<Scalar>();
    lin_vel_coeff_ = cfg["rl"]["lin_vel_coeff"].as<Scalar>();
    ang_vel_coeff_ = cfg["rl"]["ang_vel_coeff"].as<Scalar>();
    act_coeff_ = cfg["rl"]["act_coeff"].as<Scalar>();
  } else {
    return false;
  }
  return true;
}

bool QuadrotorHoverEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorHoverEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorHoverEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorHoverEnv &quad_env) {
  os.precision(3);
  os << "Quadrotor Environment:\n"
     << "obs dim =            [" << quad_env.obs_dim_ << "]\n"
     << "act dim =            [" << quad_env.act_dim_ << "]\n"
     << "sim dt =             [" << quad_env.sim_dt_ << "]\n"
     << "max_t =              [" << quad_env.max_t_ << "]\n"
     << "act_mean =           [" << quad_env.act_mean_.transpose() << "]\n"
     << "act_std =            [" << quad_env.act_std_.transpose() << "]\n"
     << "obs_mean =           [" << quad_env.obs_mean_.transpose() << "]\n"
     << "obs_std =            [" << quad_env.obs_std_.transpose() << std::endl;
  os.precision();
  return os;
}

}  // namespace flightlib