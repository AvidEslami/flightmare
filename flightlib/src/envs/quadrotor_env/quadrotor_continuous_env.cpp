#include "flightlib/envs/quadrotor_env/quadrotor_continuous_env.hpp"

namespace flightlib {

int waypoint_num_continuous = 0;

flightlib::Timer myTimer_continuous("myTimer_continuous", "starter_module");

flightlib::Scalar time_elapsed_continuous;

bool terminal_reached_continuous = false;

int flightpath = 3;
// 0: Square Path
// 1: 9 Meter
// 2: 15 Meter

int line_counter = 40;
int eff_line_counter = 40;

bool toced_continuous = false;

bool completed_lap = false;

bool assist = false;

QuadrotorContinuousEnv::QuadrotorContinuousEnv()
  : QuadrotorContinuousEnv(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_continuous_test_env.yaml")) {}

QuadrotorContinuousEnv::QuadrotorContinuousEnv(const std::string &cfg_path)
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
  world_box_ << -30, 30, -30, 30, -10, 30;
  if (!quadrotor_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  // obs_dim_ = quadenv::kNObs * 2;

  // NEW APPROACH: Let us just focus on relative positions
  obs_dim_ = (quadenv::kNObs * 2) - 3;


  act_dim_ = quadenv::kNAct;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);
}

QuadrotorContinuousEnv::~QuadrotorContinuousEnv() {}

bool QuadrotorContinuousEnv::reset(Ref<Vector<>> obs, const bool random) {
  if (terminal_reached_continuous == false) {
    // toc here if num trial > 0
    if (!toced_continuous) {
      if (waypoint_num_continuous > 0) {
        myTimer_continuous.toc();
        time_elapsed_continuous = myTimer_continuous.last();
        std::cout << "Elapsed time: (1)" << time_elapsed_continuous << std::endl;
      }
      toced_continuous = true;
    }

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

    // if (waypoint_num_continuous == 0) {
    //   goal_state_(QS::POSX) = 3.0;
    // }
    // else if (waypoint_num_continuous == 1) {
    //   goal_state_(QS::POSX) = 3.0;
    // }
    // else if (waypoint_num_continuous == 2) {
    //   goal_state_(QS::POSX) = 3.0;
    // }
    // else {
    //   goal_state_(QS::POSX) = 10.0;
    // }
    // goal_state_(QS::POSY) = 0.0;
    // goal_state_(QS::POSZ) = 3.0;
    if (random) {

      quad_state_.setZero();
      // quad_state_.x(QS::POSZ) = 7.0;
      if (false) {
        quad_state_.x(QS::POSX) = -4.99;
        quad_state_.x(QS::POSY) = 4.49;
        quad_state_.x(QS::POSZ) = 1.20+5;
        quad_state_.x(QS::ATTW) = 0.99;
        quad_state_.x(QS::ATTX) = 0.03; 
        quad_state_.x(QS::ATTY) = 0.008;
        quad_state_.x(QS::ATTZ) = 0.001;
        quad_state_.x(QS::VELX) = 0.002;
        quad_state_.x(QS::VELY) = -0.01;
        quad_state_.x(QS::VELZ) = 0.12;
        quad_state_.x(QS::OMEX) = 4.49;
        quad_state_.x(QS::OMEY) = 1.04;
        quad_state_.x(QS::OMEZ) = 0.2;
        quad_state_.x(QS::ACCX) = 0.05;
        quad_state_.x(QS::ACCY) = -0.36;
        quad_state_.x(QS::ACCZ) = 3.79;
        quad_state_.x(QS::TAUX) = 138.4;
        quad_state_.x(QS::TAUY) = 32.0;
        quad_state_.x(QS::TAUZ) = 6.24;
      }
      else {
        // quad_state_.x(QS::POSX) = 0.0659841;
        // quad_state_.x(QS::POSY) = 9.99912;
        // quad_state_.x(QS::POSZ) = 2.5+5;
        // quad_state_.x(QS::ATTW) = -0.175013;
        // quad_state_.x(QS::ATTX) = 0; 
        // quad_state_.x(QS::ATTY) = 1;
        // quad_state_.x(QS::ATTZ) = 6.59803;
        // quad_state_.x(QS::VELX) = 6.6;
        // quad_state_.x(QS::VELY) = -0.088;
        // quad_state_.x(QS::VELZ) = 0;
        quad_state_.x(QS::POSX) = -5;
        quad_state_.x(QS::POSY) = 4.5;
        quad_state_.x(QS::POSZ) = 1.2+5;
        quad_state_.x(QS::ATTW) = 0.18513;
        quad_state_.x(QS::ATTX) = 0.0;
        quad_state_.x(QS::ATTY) = 0.0;
        quad_state_.x(QS::ATTZ) = 0.98271;
        quad_state_.x(QS::VELX) = 0.0;
        quad_state_.x(QS::VELY) = 0.0;
        quad_state_.x(QS::VELZ) = 0.0;
      }
      if (flightpath == 0) {
        goal_state_(QS::POSX) = 5;
      }
      else if (flightpath == 1) {
        goal_state_(QS::POSX) = 2.9241;
        goal_state_(QS::POSZ) = 7-0.05604;
      }
      else if (flightpath == 2) {
        goal_state_(QS::POSX) = 4.0132;
        goal_state_(QS::POSZ) = 7-0.78509;
      }
      // if (waypoint_num_continuous == 0) {
      //   goal_state_(QS::POSX) = 0.0;
      // }
      // else if (waypoint_num_continuous == 1) {
      //   goal_state_(QS::POSX) = 3.0;
      // }
      // else if (waypoint_num_continuous == 2) {
      //   goal_state_(QS::POSX) = 6.0;
      // }
      // else if (waypoint_num_continuous == 3) {
      //   goal_state_(QS::POSX) = 15.0;
      // }
      // else {
      //   goal_state_(QS::POSX) = 0.0;
      // }

      // goal_state_(QS::POSY) = 0.0;
      // goal_state_(QS::POSZ) = 7.0;
      if (false) {
        goal_state_(QS::POSX) = -4.99;
        goal_state_(QS::POSY) = 4.49;
        goal_state_(QS::POSZ) = 1.20+5;
        goal_state_(QS::ATTW) = 0.99;
        goal_state_(QS::ATTX) = 0.03; 
        goal_state_(QS::ATTY) = 0.008;
        goal_state_(QS::ATTZ) = 0.001;
        goal_state_(QS::VELX) = 0.002;
        goal_state_(QS::VELY) = -0.01;
        goal_state_(QS::VELZ) = 0.12;
        goal_state_(QS::OMEX) = 4.49;
        goal_state_(QS::OMEY) = 1.04;
        goal_state_(QS::OMEZ) = 0.2;
      }
      else {
        // goal_state_(QS::POSX) = 0.526912;
        // goal_state_(QS::POSY) = 9.94405;
        // goal_state_(QS::POSZ) = 2.5+5;
        // goal_state_(QS::ATTW) = -1.39747;
        // goal_state_(QS::ATTX) = 0; 
        // goal_state_(QS::ATTY) = 1;
        // goal_state_(QS::ATTZ) = 6.56202;
        // goal_state_(QS::VELX) = 6.5664;
        // goal_state_(QS::VELY) = -1.31;
        // goal_state_(QS::VELZ) = 0;
        goal_state_(QS::POSX) = -5;
        goal_state_(QS::POSY) = 4.5;
        goal_state_(QS::POSZ) = 1.2+5;
        goal_state_(QS::ATTW) = 0.88025;
        goal_state_(QS::ATTX) = 0.0;
        goal_state_(QS::ATTY) = 0.0;
        goal_state_(QS::ATTZ) = 0.474801;
        goal_state_(QS::VELX) = 0.0;
        goal_state_(QS::VELY) = 0.0;
        goal_state_(QS::VELZ) = 0.0;
      }
      // goal_state_(QS::ACCX) = 0.05;
      // goal_state_(QS::ACCY) = -0.36;
      // goal_state_(QS::ACCZ) = 3.79;
    

    }
    

    waypoint_num_continuous = 1;
    std::cout << "waypoint_num_continuous: " << waypoint_num_continuous << std::endl;
    // Print Starting Position and Goal Position
    std::cout << "Starting Position: " << quad_state_.x(QS::POSX) << ", " << quad_state_.x(QS::POSY) << ", " << quad_state_.x(QS::POSZ) << std::endl;
    std::cout << "Goal Position: " << goal_state_(QS::POSX) << ", " << goal_state_(QS::POSY) << ", " << goal_state_(QS::POSZ) << std::endl;
    // reset quadrotor with random states

    // tic (start timer)
    toced_continuous = false;
    myTimer_continuous.tic();
    quadrotor_ptr_->reset(quad_state_);

    // reset control command
    cmd_.t = 0.0;
    cmd_.thrusts.setZero();

    // obtain observations
    getObs(obs);
    return true;
  }
  else {
    terminal_reached_continuous = false;
    myTimer_continuous.tic();
    return true;
  }
}


bool QuadrotorContinuousEnv::resetRange(Ref<Vector<>> obs, int lower_zbound, int upper_zbound, int lower_xybound, int upper_xybound, const bool random) {
  return true;
}


bool QuadrotorContinuousEnv::getObs(Ref<Vector<>> obs) {
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
  std::cout << "Current Goal State: " << goal_state_(QS::POSX) << ", " << goal_state_(QS::POSY) << ", " << goal_state_(QS::POSZ) << std::endl;
  // Print current quadrotor state
  std::cout << "Current Quadrotor State: " << quad_obs_(QS::POSX) << ", " << quad_obs_(QS::POSY) << ", " << quad_obs_(QS::POSZ) << std::endl;

  obs.segment<quadenv::kNPos>(quadenv::kPos) = goal_state_.segment<quadenv::kNPos>(quadenv::kPos) - quad_obs_.segment<quadenv::kNPos>(quadenv::kPos);
  obs.segment<quadenv::kNOri>(quadenv::kOri) = quad_obs_.segment<quadenv::kNOri>(quadenv::kOri);
  obs.segment<quadenv::kNLinVel>(quadenv::kLinVel) = quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  obs.segment<quadenv::kNAngVel>(quadenv::kAngVel) = quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  obs.segment<quadenv::kNOri>(quadenv::kOri + 9) = goal_state_.segment<quadenv::kNOri>(quadenv::kOri);
  obs.segment<quadenv::kNLinVel>(quadenv::kLinVel + 9) = goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel);
  obs.segment<quadenv::kNAngVel>(quadenv::kAngVel + 9) = goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel);

  // Print relative position and velocity and other states
  // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  // std::cout << "Relative Position: " << obs(0) << ", " << obs(1) << ", " << obs(2) << std::endl;
  // std::cout << "Relative Velocity: " << obs(3) << ", " << obs(4) << ", " << obs(5) << std::endl;
  // std::cout << "Relative Angular Velocity: " << obs(6) << ", " << obs(7) << ", " << obs(8) << std::endl;
  // std::cout << "Relative Orientation: " << obs(9) << ", " << obs(10) << ", " << obs(11) << std::endl;

  return true;
}

Scalar QuadrotorContinuousEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
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

void loadCSV(std::vector<std::string>& flight_data, std::string csv_path){
  std::fstream fs_csv;
  fs_csv.open(csv_path, std::ios::in);
  if(fs_csv.fail()){
    std::cout<<"Fail to open csv"<<std::endl;
  }
  std::string buffer;
  while(std::getline(fs_csv, buffer)){
    flight_data.push_back(buffer);
  }
}

std::vector<double> setFlightPath(int flight_path, int& waypoint_num_continuous, int& line_count, std::vector<std::string> csvFile){
  std::vector<double> flight_coords;
  switch(flight_path){
    case 0:
      switch(waypoint_num_continuous){
        case 1: 
          flight_coords = {std::numeric_limits<double>::max(), 5, std::numeric_limits<double>::max()};
          waypoint_num_continuous++;
          break;
        case 2:
          flight_coords = {0, std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
          waypoint_num_continuous++;
          break;
        case 3:
          flight_coords = {std::numeric_limits<double>::max(), 0, std::numeric_limits<double>::max()};
          waypoint_num_continuous++;
          break;
        case 4:
          flight_coords = {5, std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
          waypoint_num_continuous = 1;
          break;
      }
      break;
    case 1:
      switch(waypoint_num_continuous){
        case 1:
          flight_coords = {6.2708, std::numeric_limits<double>::max(), 7+-0.042849};
          waypoint_num_continuous++;
          break;
        case 2:
          flight_coords = {9, std::numeric_limits<double>::max(), 7};
          waypoint_num_continuous++;
          break;
        case 3:
          flight_coords = {0, std::numeric_limits<double>::max(), 7};
          waypoint_num_continuous = 1;
          break;
      }
      break;
    case 2:
      switch(waypoint_num_continuous){
        case 1:
          flight_coords = {8.0129, std::numeric_limits<double>::max(), 7-1.3501};
          waypoint_num_continuous++;
          break;
        case 2:
          flight_coords = {12.065, std::numeric_limits<double>::max(), 7-1.346};
          waypoint_num_continuous++;
          break;
        case 3:
          flight_coords = {16.043, std::numeric_limits<double>::max(), 7-0.78063};
          waypoint_num_continuous++;
          break;
        case 4:
          flight_coords = {20, std::numeric_limits<double>::max(), 7};
          waypoint_num_continuous++;
          break;
        case 5:
          flight_coords = {0, std::numeric_limits<double>::max(), 7};
          waypoint_num_continuous = 1;
          break;
      }
      break;
    case 3:
      if(line_count >= csvFile.size()){
        line_count = 10;
        waypoint_num_continuous = 1;
        completed_lap = true;
      }
      std::string line = csvFile[line_count];
      std::stringstream line_stream(line);
      for(int i = 0; i < 10; i++){
        std::string data;
        std::getline(line_stream, data, ',');
        if(i == 0){
          continue;
        }
        else{
          flight_coords.push_back(stod(data));
        }
      }
      line_count += eff_line_counter;
      waypoint_num_continuous++;
      if(line_count > csvFile.size()){
        waypoint_num_continuous = 1;
        completed_lap = true;
      }
      break;
  }
  return flight_coords;
}


bool QuadrotorContinuousEnv::isTerminalState(Scalar &reward) {
  if (completed_lap) {
    printf("Lap Completed!!!!!!!!");
    return true;
  }
  // if (quad_state_.x(QS::POSZ) <= 0.02) {
  //   reward = -0.02;
  //   if (!toced_continuous) {
  //     myTimer_continuous.toc();
  //     time_elapsed_continuous = myTimer_continuous.last();
  //     std::cout << "Elapsed time: (2)" << time_elapsed_continuous << std::endl;
  //     toced_continuous = true;
  //     terminal_reached_continuous = true;
  //   }
  //   return true;
  // }
  // We want the quadrotor to terminate within 0.1m of the goal, and reward it immediately for doing so
  if (((quad_state_.x.segment<quadenv::kNPos>(quadenv::kPos) -
       goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
        .squaredNorm() < 0.9)) { // Temporarily increased to 0.1
    reward = 10.0;
    myTimer_continuous.toc();
    time_elapsed_continuous = myTimer_continuous.last();
    std::cout << "Elapsed time: (2)" << time_elapsed_continuous << std::endl;
    // toced_continuous = true;
    // terminal_reached_continuous = true;
    // return true;
    // if (flightpath == 0) {
    //   if (waypoint_num_continuous == 1) {
    //     waypoint_num_continuous = 2;
    //     goal_state_(QS::POSY) = 5;
    //   }
    //   else if (waypoint_num_continuous == 2) {
    //     waypoint_num_continuous = 3;
    //     goal_state_(QS::POSX) = 0;
    //   }
    //   else if (waypoint_num_continuous == 3) {
    //     waypoint_num_continuous = 4;
    //     goal_state_(QS::POSY) = 0;
    //   }
    //   else if (waypoint_num_continuous == 4) {
    //     waypoint_num_continuous = 1;
    //     goal_state_(QS::POSX) = 5;
    //   }
    //   else {
    //     toced_continuous = true;
    //     terminal_reached_continuous = true;
    //     return true;
    //   }
    // }
    // else if (flightpath == 1) {
    //   if (waypoint_num_continuous == 1) {
    //     waypoint_num_continuous = 2;
    //     goal_state_(QS::POSX) = 6.2708;
    //     goal_state_(QS::POSZ) = 7+-0.042849;
    //   }
    //   else if (waypoint_num_continuous == 2) {
    //     waypoint_num_continuous = 3;
    //     goal_state_(QS::POSX) = 9;
    //     goal_state_(QS::POSZ) = 7;
    //   }
    //   else if (waypoint_num_continuous == 3) {
    //     waypoint_num_continuous = 1;
    //     goal_state_(QS::POSX) = 0;
    //     goal_state_(QS::POSZ) = 7;
    //   }
    // }
    // else if (flightpath == 2) {
    //   if (waypoint_num_continuous == 1) {
    //     waypoint_num_continuous = 2;
    //     goal_state_(QS::POSX) = 8.0129;
    //     goal_state_(QS::POSZ) = 7-1.3501;
    //   }
    //   else if (waypoint_num_continuous == 2) {
    //     waypoint_num_continuous = 3;
    //     goal_state_(QS::POSX) = 12.065;
    //     goal_state_(QS::POSZ) = 7-1.346;
    //   }
    //   else if (waypoint_num_continuous == 3) {
    //     waypoint_num_continuous = 4;
    //     goal_state_(QS::POSX) = 16.043;
    //     goal_state_(QS::POSZ) = 7-0.78063;
    //   }
    //   else if (waypoint_num_continuous == 4) {
    //     waypoint_num_continuous = 5;
    //     goal_state_(QS::POSX) = 20;
    //     goal_state_(QS::POSZ) = 7;
    //   }
    //   else if (waypoint_num_continuous == 5) {
    //     waypoint_num_continuous = 1;
    //     goal_state_(QS::POSX) = 0;
    //     goal_state_(QS::POSZ) = 7;
    //   }
    // }
    // std::string csv_path = "/home/avidavid/Downloads/CPC16_Z1.csv";
    // std::string csv_path = "/home/avidavid/Downloads/data01.csv";
    std::string csv_path = "/home/avidavid/Downloads/big_circle.csv";
    std::vector<std::string> track_data;
    std::vector<double> coordinates;
    loadCSV(track_data, csv_path);
    std::cout<<"Number of flights: "<<track_data.size()<<std::endl;
    coordinates = setFlightPath(flightpath, waypoint_num_continuous, line_counter, track_data);

    if (assist == true) {
      // Set the current state to what the goal state was
      // quad_state_.setZero();
      // quad_state_.x.setZero();
      // quad_act_.setZero();
      quad_state_.x(QS::POSX) = goal_state_(QS::POSX);
      quad_state_.x(QS::POSY) = goal_state_(QS::POSY);
      quad_state_.x(QS::POSZ) = goal_state_(QS::POSZ);
      quad_state_.x(QS::ATTW) = goal_state_(QS::ATTW);
      quad_state_.x(QS::ATTX) = goal_state_(QS::ATTX);
      quad_state_.x(QS::ATTY) = goal_state_(QS::ATTY);
      quad_state_.x(QS::ATTZ) = goal_state_(QS::ATTZ);
      quad_state_.x(QS::VELX) = goal_state_(QS::VELX);
      quad_state_.x(QS::VELY) = goal_state_(QS::VELY);
      quad_state_.x(QS::VELZ) = goal_state_(QS::VELZ);
      // quad_state_.x(QS::OMEX) = goal_state_(QS::OMEX);
      // quad_state_.x(QS::OMEY) = goal_state_(QS::OMEY);
      // quad_state_.x(QS::OMEZ) = goal_state_(QS::OMEZ);
      // quad_state_.x(QS::ACCX) = goal_state_(QS::ACCX);
      // quad_state_.x(QS::ACCY) = goal_state_(QS::ACCY);
      // quad_state_.x(QS::ACCZ) = goal_state_(QS::ACCZ);
      quadrotor_ptr_->reset(quad_state_);
      // cmd_.t = 0.0;
      // cmd_.thrusts.setZero();
      // getObs(obs);
    }

    if(coordinates[0] != std::numeric_limits<double>::max()){
      goal_state_(QS::POSX) = coordinates[0];
    }
    if(coordinates[1] != std::numeric_limits<double>::max()){
      goal_state_(QS::POSY) = coordinates[1];
    }
    if(coordinates[2] != std::numeric_limits<double>::max()){
      goal_state_(QS::POSZ) = coordinates[2]+5;
    }
    if(coordinates[3] != std::numeric_limits<double>::max()){
      goal_state_(QS::ATTW) = coordinates[3];
    }
    if(coordinates[4] != std::numeric_limits<double>::max()){
      goal_state_(QS::ATTX) = coordinates[4];
    }
    if(coordinates[5] != std::numeric_limits<double>::max()){
      goal_state_(QS::ATTY) = coordinates[5];
    }
    if(coordinates[6] != std::numeric_limits<double>::max()){
      goal_state_(QS::ATTZ) = coordinates[6];
    }
    if(coordinates[7] != std::numeric_limits<double>::max()){
      goal_state_(QS::VELX) = coordinates[7];
    }
    if(coordinates[8] != std::numeric_limits<double>::max()){
      goal_state_(QS::VELY) = coordinates[8];
    }
    if(coordinates[9] != std::numeric_limits<double>::max()){
      goal_state_(QS::VELZ) = coordinates[9];
    }
    // if(coordinates[10] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::OMEX) = coordinates[10];
    // }
    // if(coordinates[11] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::OMEY) = coordinates[11];
    // }
    // if(coordinates[12] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::OMEZ) = coordinates[12];
    // }
    // if(coordinates[13] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::ACCX) = coordinates[13];
    // }
    // if(coordinates[14] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::ACCY) = coordinates[14];
    // }
    // if(coordinates[15] != std::numeric_limits<double>::max()){
    //   goal_state_(QS::ACCZ) = coordinates[15];
    // }
  }
  reward = 0.0;
  return false;
}

bool QuadrotorContinuousEnv::loadParam(const YAML::Node &cfg) {
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

bool QuadrotorContinuousEnv::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorContinuousEnv::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorContinuousEnv::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorContinuousEnv &quad_env) {
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