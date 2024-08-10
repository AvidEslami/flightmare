#include "flightlib/envs/quadrotor_env/quadrotor_angled_env.hpp"

namespace flightlib {

QuadrotorAngledEnv::QuadrotorAngledEnv()
  : QuadrotorAngledEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

QuadrotorAngledEnv::QuadrotorAngledEnv(const std::string &cfg_path)
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
  world_box_ << -30, 30, -30, 30, -30, 30;
  if (!quadrotor_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  obs_dim_ = (quadenv::kNObs * 2) - 3;

  // // NEW APPROACH: Let us just focus on relative positions
  // obs_dim_ = quadenv::kNObs;


  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);
}

QuadrotorAngledEnv::~QuadrotorAngledEnv() {}

bool QuadrotorAngledEnv::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

  if (random) {
    // randomly reset the quadrotor state
    // reset position
    std::uniform_real_distribution<Scalar> velocity_init(-5, 5);
    // std::uniform_real_distribution<Scalar> orientation_w_init(0.884, 0.919);
    // std::uniform_real_distribution<Scalar> orientation_x_init(-0.177, 0.177);
    // std::uniform_real_distribution<Scalar> orientation_y_init(-0.306, 0.306);
    // std::uniform_real_distribution<Scalar> orientation_z_init(-0.306, 0.177);

    quad_state_.setZero();
    goal_state_.setZero();
    quad_state_.x(QS::POSZ) = 10;
    // Give drone a random starting velocity between -1 and 1 m/s in x, y, and z
    quad_state_.x(QS::VELX) = 0;
    quad_state_.x(QS::VELY) = 0;
    quad_state_.x(QS::VELZ) = 0;

    // // Orient the drone in the direction of its velocity
    quad_state_.x(QS::ATTW) = 1;
    quad_state_.x(QS::ATTX) = 0;
    quad_state_.x(QS::ATTY) = 0;
    quad_state_.x(QS::ATTZ) = 0;
    quad_state_.x(QS::OMEX) = 0;
    quad_state_.x(QS::OMEY) = 0;
    quad_state_.x(QS::OMEZ) = 0;

    // reset linear velocity
    // quad_state_.x(QS::VELX) = velocity_init(random_gen_);
    // quad_state_.x(QS::VELY) = velocity_init(random_gen_);
    // quad_state_.x(QS::VELZ) = velocity_init(random_gen_);
    // reset orientation
    // quad_state_.x(QS::ATTW) = orientation_w_init(random_gen_);
    // quad_state_.x(QS::ATTX) = orientation_x_init(random_gen_);
    // quad_state_.x(QS::ATTY) = orientation_y_init(random_gen_);
    // quad_state_.x(QS::ATTZ) = orientation_z_init(random_gen_);
    // quad_state_.qx /= quad_state_.qx.norm();
    
    std::uniform_real_distribution<Scalar> altitude_dist(8, 12);
    std::uniform_real_distribution<Scalar> xy_dist(-2, 2);
    // std::uniform_real_distribution<Scalar> orientation_w(0.884, 0.919);
    // std::uniform_real_distribution<Scalar> orientation_x(-0.177, 0.177);
    // std::uniform_real_distribution<Scalar> orientation_y(-0.306, 0.306);
    // std::uniform_real_distribution<Scalar> orientation_z(-0.306, 0.177);

    goal_state_(QS::POSX) = xy_dist(random_gen_);
    goal_state_(QS::POSY) = xy_dist(random_gen_);
    goal_state_(QS::POSZ) = altitude_dist(random_gen_);

    goal_state_(QS::VELX) = velocity_init(random_gen_);
    goal_state_(QS::VELY) = velocity_init(random_gen_);
    goal_state_(QS::VELZ) = velocity_init(random_gen_);

    // Assign the velocity we would like the drone to reach at the goal state
    float x_velocity = goal_state_(QS::VELX);
    float y_velocity = goal_state_(QS::VELY);
    float z_velocity = goal_state_(QS::VELZ);
    double magnitude = std::sqrt(x_velocity * x_velocity + y_velocity * y_velocity + z_velocity * z_velocity);
    x_velocity /= magnitude;
    y_velocity /= magnitude;
    z_velocity /= magnitude;
    float roll = std::atan2(y_velocity, std::sqrt(x_velocity*x_velocity + z_velocity*z_velocity));
    float pitch = std::atan2(-z_velocity, std::sqrt(x_velocity*x_velocity + y_velocity*y_velocity));
    float yaw = std::atan2(y_velocity, x_velocity);
    goal_state_(QS::ATTW) = std::cos(yaw/2)*std::cos(pitch/2)*std::cos(roll/2)+std::sin(yaw/2)*std::sin(pitch/2)*std::sin(roll/2);
    goal_state_(QS::ATTX) = std::cos(yaw/2)*std::cos(pitch/2)*std::sin(roll/2)-std::sin(yaw/2)*std::sin(pitch/2)*std::cos(roll/2);
    goal_state_(QS::ATTY) = std::sin(yaw/2)*std::cos(pitch/2)*std::sin(roll/2)+std::cos(yaw/2)*std::sin(pitch/2)*std::cos(roll/2);
    goal_state_(QS::ATTZ) = std::sin(yaw/2)*std::cos(pitch/2)*std::cos(roll/2)-std::cos(yaw/2)*std::sin(pitch/2)*std::sin(roll/2);
    goal_state_(QS::OMEX) = 0;
    goal_state_(QS::OMEY) = 0;
    goal_state_(QS::OMEZ) = 0;

    // // Ensure the desired goal state direction is roughly in the direction of the drone's velocity (vary by less than 90 degrees)
    // int direction_changes = 0;
    // if (goal_state_(QS::VELX)*quad_state_.x(QS::VELX) < 0){
    //   direction_changes++;
    // }
    // if (goal_state_(QS::VELY)*quad_state_.x(QS::VELY) < 0){
    //   direction_changes++;
    // }
    // if (goal_state_(QS::VELZ)*quad_state_.x(QS::VELZ) < 0){
    //   direction_changes++;
    // }

    // while (direction_changes > 1){
    //   goal_state_(QS::POSX) = xy_dist(random_gen_);
    //   goal_state_(QS::POSY) = xy_dist(random_gen_);
    //   goal_state_(QS::POSZ) = altitude_dist(random_gen_);
    //   goal_state_(QS::VELX) = velocity_init(random_gen_);
    //   goal_state_(QS::VELY) = velocity_init(random_gen_);
    //   goal_state_(QS::VELZ) = velocity_init(random_gen_);
    //   direction_changes = 0;
    //   if (goal_state_(QS::VELX)*quad_state_.x(QS::VELX) < 0){
    //     direction_changes++;
    //   }
    //   if (goal_state_(QS::VELY)*quad_state_.x(QS::VELY) < 0){
    //     direction_changes++;
    //   }
    //   if (goal_state_(QS::VELZ)*quad_state_.x(QS::VELZ) < 0){
    //     direction_changes++;
    //   }
    // }

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

bool QuadrotorAngledEnv::resetRange(Ref<Vector<>> obs, int lower_zbound, int upper_zbound, int lower_xybound, int upper_xybound, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();

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
    
    std::uniform_real_distribution<Scalar> altitude_dist(lower_zbound, upper_zbound);
    std::uniform_real_distribution<Scalar> xy_dist(lower_xybound, upper_xybound);

    goal_state_(QS::POSX) = xy_dist(random_gen_);
    goal_state_(QS::POSY) = xy_dist(random_gen_);
    goal_state_(QS::POSZ) = altitude_dist(random_gen_);

    // goal_state_(QS::VELX) = velocity_init(random_gen_);
    // goal_state_(QS::VELY) = velocity_init(random_gen_);
    // goal_state_(QS::VELZ) = velocity_init(random_gen_);

    // Assign the velocity we would like the drone to reach at the goal state
    // float x_velocity = goal_state_(QS::VELX);
    // float y_velocity = goal_state_(QS::VELY);
    // float z_velocity = goal_state_(QS::VELZ);
    // double magnitude = std::sqrt(x * x + y * y + z * z);
    // x_velocity /= magnitude;
    // y_velocity /= magnitude;
    // z_velocity /= magnitude;
    // float roll = std::atan2(y_velocity, std::sqrt(x_velocity*x_velocity + z_velocity*z_velocity));
    // float pitch = std::atan2(-z_velocity, std::sqrt(x_velocity*x_velocity + y_velocity*y_velocity));
    // float yaw = std::atan2(y_velocity, x_velocity);
    // goal_state_(QS::ATTW) = std::cos(yaw/2)*std::cos(pitch/2)*std::cos(roll/2)+std::sin(yaw/2)*std::sin(pitch/2)*std::sin(roll/2);
    // goal_state_(QS::ATTX) = std::cos(yaw/2)*std::cos(pitch/2)*std::sin(roll/2)-std::sin(yaw/2)*std::sin(pitch/2)*std::cos(roll/2);
    // goal_state_(QS::ATTY) = std::sin(yaw/2)*std::cos(pitch/2)*std::sin(roll/2)+std::cos(yaw/2)*std::sin(pitch/2)*std::cos(roll/2);
    // goal_state_(QS::ATTZ) = std::sin(yaw/2)*std::cos(pitch/2)*std::cos(roll/2)-std::cos(yaw/2)*std::sin(pitch/2)*std::sin(roll/2);
    // goal_state_(QS::OMEX) = 0;
    // goal_state_(QS::OMEY) = 0;
    // goal_state_(QS::OMEZ) = 0;
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

// bool QuadrotorAngledEnv::getObs(Ref<Vector<>> obs) {
//   quadrotor_ptr_->getState(&quad_state_);

//   // convert quaternion to euler angle
//   Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
//   // quaternionToEuler(quad_state_.q(), euler);
//   quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w, goal_state_;

//   // obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
//   return true;
// }

// bool QuadrotorAngledEnv::getObs(Ref<Vector<>> obs) {
//   quadrotor_ptr_->getState(&quad_state_);

//   // convert quaternion to euler angle
//   Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
//   // quaternionToEuler(quad_state_.q(), euler);
//   quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w;

//   obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
//   obs.segment<quadenv::kNObs>(quadenv::kObs + quadenv::kNObs) = goal_state_;
//   // obs(quadenv::kNObs) = goal_state_(QS::POSZ); // add goal state to observation vector


//   // NEW APPROACH: Let us just focus on relative positions
//   //               This lets us reduce model size and improve performance/generalization speed
//   // obs.segment<quadenv::kNObs>(quadenv::kObs) = goal_state_.segment<quadenv::kNObs>(quadenv::kObs) - quad_obs_.segment<quadenv::kNObs>(quadenv::kObs);

//   return true;
// }

bool QuadrotorAngledEnv::getObs(Ref<Vector<>> obs) {
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
  obs.segment<quadenv::kNPos>(quadenv::kPos) = goal_state_.segment<quadenv::kNPos>(quadenv::kPos) - quad_obs_.segment<quadenv::kNPos>(quadenv::kPos);
  obs.segment<quadenv::kNOri>(quadenv::kOri) = quad_obs_.segment<quadenv::kNOri>(quadenv::kOri);
  obs.segment<quadenv::kNLinVel>(quadenv::kLinVel) = quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  obs.segment<quadenv::kNAngVel>(quadenv::kAngVel) = quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  obs.segment<quadenv::kNOri>(quadenv::kOri + 9) = goal_state_.segment<quadenv::kNOri>(quadenv::kOri);
  obs.segment<quadenv::kNLinVel>(quadenv::kLinVel + 9) = goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  obs.segment<quadenv::kNAngVel>(quadenv::kAngVel + 9) = goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel);


  // Print Full Drone State
  // std::cout << quad_obs_.transpose() << std::endl;
  // // Print Full Goal State
  // std::cout << goal_state_.transpose() << std::endl;
  // // Print Full Observation
  // std::cout << obs.transpose() << std::endl;
  // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // New ObsDim is 12

  return true;
}

Scalar QuadrotorAngledEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
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
                   .squaredNorm();
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
    pos_reward + act_reward + ori_reward + ang_vel_reward + lin_vel_reward;

  // survival reward
  total_reward += 0.15;

  return total_reward;
}

bool QuadrotorAngledEnv::isTerminalState(Scalar &reward) {
  if ((((quad_state_.x.segment<quadenv::kNPos>(quadenv::kPos) -
       goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
        .squaredNorm() < 0.02))) {
    // We want the quadrotor to terminate within 0.1m of the goal, and reward it immediately for doing so
    // double dist = (quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) - goal_state_.segment<quadenv::kNPos>(quadenv::kPos)).squaredNorm();
    // double power = -0.5*std::pow(dist/0.5, 2);
    // reward = 10.0*std::exp(power);
    // reward = 30.0;
    // reward = 0;

    // // Use a bell curve to reward the drone for having a velocity that is very close to the desired velocity
    // // MAXIMUM REWARD FROM VELOCITY: 40.0, MINIMUM REWARD FROM VELOCITY: 0.0
    double vel_dist = (quad_state_.x.segment<quadenv::kNLinVel>(quadenv::kLinVel) - goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel)).squaredNorm();
    double vel_power = -0.5*std::pow(vel_dist/0.5, 2);
    reward += 50.0*std::exp(vel_power);


    // Use a bell curve to reward the drone for having a terminal orientation that is very close to the desired orientation
    // MAXIMUM REWARD FROM ORIENTATION: 40.0, MINIMUM REWARD FROM ORIENTATION: 0.0
    double ori_dist = (quad_state_.x.segment<quadenv::kNOri>(quadenv::kOri) - goal_state_.segment<quadenv::kNOri>(quadenv::kOri)).squaredNorm();
    double ori_power = -0.5*std::pow(ori_dist/0.5, 2);
    reward += 50.0*std::exp(ori_power);
    std::cout << "Terminal Reward: " << reward << std::endl;
    return true;
  }
  else if ((quad_state_.x(QS::POSZ) <= -10.0)) {
    reward = -50.5;
    return true;
  }
  else {
    // reward = 0.0;
    return false;
  }
}

bool QuadrotorAngledEnv::loadParam(const YAML::Node &cfg) {
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

bool QuadrotorAngledEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorAngledEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorAngledEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorAngledEnv &quad_env) {
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