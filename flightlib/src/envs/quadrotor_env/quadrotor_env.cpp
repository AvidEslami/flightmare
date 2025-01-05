#include "flightlib/envs/quadrotor_env/quadrotor_env.hpp"

namespace flightlib {

QuadrotorEnv::QuadrotorEnv()
  : QuadrotorEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env_kingfisher.yaml")) {}

int debug_hover_commands = 1;
int hover_dynamics_randomization = 0;
int hover_debug_dynamics = 1;

QuadrotorEnv::QuadrotorEnv(const std::string &cfg_path)
  : EnvBase(),
    pos_coeff_(0.0),
    ori_coeff_(0.0),
    lin_vel_coeff_(0.0),
    ang_vel_coeff_(0.0),
    act_coeff_(0.0),
    goal_state_((Vector<quadenv::kNObs>() << 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0,
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

  original_dynamics_ = dynamics;

  // Print the quadrotor dynamics
  std::cout << "Quadrotor Dynamics: " << dynamics << std::endl;  

  // define a bounding box
  world_box_ << -20, 20, -20, 20, 0, 20;
  if (!quadrotor_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  // obs_dim_ = quadenv::kNObs * 2;
  obs_dim_ = 12;
  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);

  // print the quadrotor environment
  std::cout << *this << std::endl;
}

QuadrotorEnv::~QuadrotorEnv() {}

bool QuadrotorEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  if (hover_dynamics_randomization) {
    QuadrotorDynamics dynamics = original_dynamics_;
    // std::cout << "Original Dynamics: " << std::endl;
    // std::cout << dynamics << std::endl;
    // Randomly change the parameters by -15% to 15%
    std::uniform_real_distribution<Scalar> random_dist(-0.10, 0.10);    
    // Now create new quadrotor_dynamics
    // mass
    // arm_l
    // motor_tau
    QuadrotorDynamics new_dynamics;
    new_dynamics.setMass(dynamics.getMass() + dynamics.getMass()*random_dist(random_gen_));
    new_dynamics.setArmLength(dynamics.getArmLength() + dynamics.getArmLength()*random_dist(random_gen_));
    new_dynamics.setMotortauInv(dynamics.getMotorTauInv() + dynamics.getMotorTauInv()*random_dist(random_gen_));

    // Slightly vary each term in the inertia matrix
    // Matrix<3, 3> inertia = dynamics.getInertia();
    // Matrix<3, 3> new_inertia;
    // for (int i = 0; i < 3; i++) {
    //   for (int j = 0; j < 3; j++) {
    //     new_inertia(i, j) = inertia(i, j) + inertia(i, j)*random_dist(random_gen_);
    //   }
    // }
    // new_dynamics.setInertia(new_inertia);

    quadrotor_ptr_->updateDynamics(new_dynamics);

    // std::cout << "Randomized Dynamics: " << std::endl;
    // std::cout << dynamics << std::endl;
  }

  if (hover_debug_dynamics) {
    std::cout << "Hover Rollout Dynamics: " << std::endl;
    std::cout << quadrotor_ptr_->getDynamics() << std::endl;
  }

  if (random) {
    // randomly reset the quadrotor state
    // reset position
    quad_state_.x(QS::POSX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::POSZ) = uniform_dist_(random_gen_) + 5;
    if (quad_state_.x(QS::POSZ) < -0.0)
      quad_state_.x(QS::POSZ) = -quad_state_.x(QS::POSZ);
    // reset linear velocity
    quad_state_.x(QS::VELX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::VELZ) = uniform_dist_(random_gen_);
    // reset orientation
    quad_state_.x(QS::ATTW) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTX) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTY) = uniform_dist_(random_gen_);
    quad_state_.x(QS::ATTZ) = uniform_dist_(random_gen_);
    quad_state_.qx /= quad_state_.qx.norm();
    
    std::uniform_real_distribution<Scalar> altitude_dist(3.0, 9.0);
    std::uniform_real_distribution<Scalar> xy_dist(-2.0, 2.0);

    goal_state_(QS::POSX) = xy_dist(random_gen_);
    goal_state_(QS::POSY) = xy_dist(random_gen_);
    goal_state_(QS::POSZ) = altitude_dist(random_gen_);
  }
  // reset quadrotor with random states
  quadrotor_ptr_->reset(quad_state_);

  // reset control command
  cmd_.t = 0.0;
  cmd_.thrusts.setZero();

  // obtain observations
  getObs(obs);
  return true;
}

bool QuadrotorEnv::resetRange(Ref<Vector<>> obs, int lower_zbound, int upper_zbound, int lower_xybound, int upper_xybound, const bool random) {
  return true;
}


// bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
//   quadrotor_ptr_->getState(&quad_state_);

//   // convert quaternion to euler angle
//   Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
//   // quaternionToEuler(quad_state_.q(), euler);
//   quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w, goal_state_;

//   // obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
//   return true;
// }

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
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
  // obs.segment<quadenv::kNObs>(quadenv::kObs) = goal_state_.segment<quadenv::kNObs>(quadenv::kObs) - quad_obs_.segment<quadenv::kNObs>(quadenv::kObs);
  

  // Print current goal state
  // std::cout << "Current Goal State: " << goal_state_(QS::POSX) << ", " << goal_state_(QS::POSY) << ", " << goal_state_(QS::POSZ) << std::endl;
  // Print relative position and velocity and other states
  // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // std::cout << "Relative Position: " << obs(0) << ", " << obs(1) << ", " << obs(2) << std::endl;
  // std::cout << "Relative Velocity: " << obs(3) << ", " << obs(4) << ", " << obs(5) << std::endl;
  // std::cout << "Relative Angular Velocity: " << obs(6) << ", " << obs(7) << ", " << obs(8) << std::endl;
  // std::cout << "Relative Orientation: " << obs(9) << ", " << obs(10) << ", " << obs(11) << std::endl;
  // Newest Approach, use relative position then append drone's velocity and orientation followed by goal state velocity and orientation
  // obs.segment<quadenv::kNPos>(quadenv::kPos) = goal_state_.segment<quadenv::kNPos>(quadenv::kPos) - quad_obs_.segment<quadenv::kNPos>(quadenv::kPos);
  // obs.segment<quadenv::kNOri>(quadenv::kOri) = quad_obs_.segment<quadenv::kNOri>(quadenv::kOri);
  // obs.segment<quadenv::kNLinVel>(quadenv::kLinVel) = quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  // obs.segment<quadenv::kNAngVel>(quadenv::kAngVel) = quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  // obs.segment<quadenv::kNOri>(quadenv::kOri + 9) = goal_state_.segment<quadenv::kNOri>(quadenv::kOri);
  // obs.segment<quadenv::kNLinVel>(quadenv::kLinVel + 9) = goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  // obs.segment<quadenv::kNAngVel>(quadenv::kAngVel + 9) = goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  // Back to old approach, relative position, orientation, linear velocity, and angular velocity
  obs.segment<quadenv::kNPos>(quadenv::kPos) = goal_state_.segment<quadenv::kNPos>(quadenv::kPos) - quad_obs_.segment<quadenv::kNPos>(quadenv::kPos);
  obs.segment<quadenv::kNOri>(quadenv::kOri) = quad_obs_.segment<quadenv::kNOri>(quadenv::kOri);
  obs.segment<quadenv::kNLinVel>(quadenv::kLinVel) = quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  obs.segment<quadenv::kNAngVel>(quadenv::kAngVel) = quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  // Print Full Drone State
  // std::cout << quad_obs_.transpose() << std::endl;
  // // Print Full Goal State
  // std::cout << goal_state_.transpose() << std::endl;
  // Print Full Observation

  if (debug_hover_commands) {
    std::cout << obs.transpose() << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  }
  // New ObsDim is 12

  return true;
}

Scalar QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  cmd_.t += sim_dt_;
  cmd_.thrusts = quad_act_;

  // Log the current act
  if (debug_hover_commands) {
    std::cout << "Current Act: " << quad_act_.transpose() << std::endl;
    std::cout << "Raw Act: " << act.transpose() << std::endl;
    std::cout << "Current act_std: " << act_std_.transpose() << std::endl;
    std::cout << "Current act_mean: " << act_mean_.transpose() << std::endl;
  }


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

bool QuadrotorEnv::isTerminalState(Scalar &reward) {
  if (quad_state_.x(QS::POSZ) <= 0.02) {
    reward = -0.02;
    return true;
  }
  reward = 0.0;
  return false;
}

bool QuadrotorEnv::loadParam(const YAML::Node &cfg) {
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

bool QuadrotorEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorEnv &quad_env) {
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