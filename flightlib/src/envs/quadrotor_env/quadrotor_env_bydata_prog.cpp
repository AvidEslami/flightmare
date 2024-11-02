#include "flightlib/envs/quadrotor_env/quadrotor_env_bydata_prog.hpp"

namespace flightlib {

QuadrotorEnvByDataProg::QuadrotorEnvByDataProg()
  : QuadrotorEnvByDataProg(getenv("FLIGHTMARE_PATH") +
                 std::string("/flightlib/configs/quadrotor_env.yaml")) {}

// std::string trajPath1 = "/home/avidavid/Downloads/CPC16_Z1.csv";
// std::string trajPath2 = "/home/avidavid/Downloads/CPC25_Z1 (1).csv";
// std::string trajPath3 = "/home/avidavid/Downloads/CPC33_Z1 (1).csv";
// std::string trajPath4 = "/home/avidavid/Downloads/random_states.csv";
// std::string trajPath5 = "/home/avidavid/Downloads/random_states (1).csv";
// std::string trajPath6 = "/home/avidavid/Downloads/0.016.csv";
// std::string trajPath7 = "/home/avidavid/Downloads/data01.csv";
// std::string trajPath8 = "/home/avidavid/Downloads/3m_vel_norm.csv";
// std::string trajPath9 = "/home/avidavid/Downloads/6m_vel_norm.csv";

// std::string cirPath1 = "/home/avidavid/Downloads/4m_circle.csv";
// std::string cirPath2 = "/home/avidavid/Downloads/6m_circle.csv";
// std::string cirPath3 = "/home/avidavid/Downloads/8m_circle.csv";

float prog_view_horizon = 0.5f;
float prog_train_horizon = 5.5f;
// Store second last state and use it for computing bell curve rewards at terminal state
// Vector<quadenv::kNObs> second_last_state;
int log_positions_to_files = 1;

int prog_debug_actions = 0;
int prog_debug_horizons = 0;
int prog_debug_velocities = 0;
int prog_debug_orientations = 0;
int prog_debug_total_reward = 0;
int prog_debug_time = 0;
int prog_debug_dynamics = 0;
int prog_debug_observations = 0;

int add_random_noise_temp = 0;

int prog_enable_orientation_reward = 0;

QuadrotorEnvByDataProg::QuadrotorEnvByDataProg(const std::string &cfg_path)
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

  if (prog_debug_dynamics) {
    std::cout << "Quadrotor dynamics: " << std::endl;
    std::cout << dynamics << std::endl;
  }

  // define a bounding box
  world_box_ << -50, 50, -50, 50, -50, 50;
  if (!quadrotor_ptr_->setWorldBox(world_box_)) {
    logger_.error("cannot set wolrd box");
  };

  // define input and output dimension for the environment
  // obs_dim_ = quadenv::kNObs * 2;

  // // NEW APPROACH: Let us just focus on relative positions
  // obs_dim_ = (quadenv::kNObs * 2) - 3;
  // NEW APPROACH: We start with drone velocity and orientation
  // Then we add the relative position from the drone to the next 50 points in the trajectory
  // obs_dim_ = int(floor(3+3+3*50));
  // obs_dim_ = int(floor(3+3+3*50*0.25)); // Pos every 4 steps
  // obs_dim_ = int(floor(3+3+3*50*0.25+3*50*0.25)); // Pos and Ori every 4 steps

  // When the trajectory is obvious:
  obs_dim_ = quadenv::kNObs;

  act_dim_ = quadenv::kNAct;

  mid_train_step_ = 0;

  Scalar mass = quadrotor_ptr_->getMass();
  act_mean_ = Vector<quadenv::kNAct>::Ones() * (-mass * Gz) / 4;
  act_std_ = Vector<quadenv::kNAct>::Ones() * (-mass * 2 * Gz) / 4;

  // load parameters
  loadParam(cfg_);

  if (log_positions_to_files) {
    // Create a csv file to log the positions, each row will be an x,y,z position
    std::ofstream logFile;
    logFile.open("positions.csv");
    logFile << "x,y,z" << std::endl;
    logFile.close();
  }
}

QuadrotorEnvByDataProg::~QuadrotorEnvByDataProg() {}



bool QuadrotorEnvByDataProg::reset(Ref<Vector<>> obs, const bool random) {
  quad_state_.setZero();
  quad_act_.setZero();
  mid_train_step_ = 0;
  traj_.clear();

  if (random) {
    // randomly reset the quadrotor state
    quad_state_.setZero();
    quad_state_.x(QS::POSZ) = uniform_dist_(random_gen_) + 10;

    // float time_partition_window = 0.5f;
    float time_partition_window = prog_view_horizon;
    float measurement_partition_window = prog_train_horizon;
    // Pick a random number to choose which data file to use
    // std::uniform_int_distribution<int> data_file_dist(1, 2);
    // int data_file_choice = data_file_dist(random_gen_);
    // Pick traj_path randomly between 8 and 9
    // std::uniform_int_distribution<int> data_file_dist(8, 9);
    // int data_file_choice = data_file_dist(random_gen_);
    // std::cout << "Data file choice: " << data_file_choice << std::endl;


    // // Pick a random circle path from 1 to 3
    // std::uniform_int_distribution<int> data_file_dist(1, 3);
    // int data_file_choice = data_file_dist(random_gen_);

    std::string trajPath;
    // if (data_file_choice == 1){
    //   trajPath = cirPath1;
    // }
    // else if (data_file_choice == 2){
    //   trajPath = cirPath2;
    // }
    // else{
    //   trajPath = cirPath3;
    // }

    trajPath = "/home/avidavid/Downloads/dummy_circle_path.csv";

    // if (data_file_choice == 1){
    //   trajPath = trajPath1;
    // }
    // else if (data_file_choice == 2){
    //   trajPath = trajPath2;
    // }
    // else if (data_file_choice == 5){
    //   trajPath = trajPath5;
    // }
    // else if (data_file_choice == 6) {
    //   trajPath = trajPath6;
    // }
    // else if (data_file_choice == 7) {
    //   trajPath = trajPath7;
    // }
    // else if (data_file_choice == 8) {
    //   trajPath = trajPath8;
    // }
    // else if (data_file_choice == 9) {
    //   trajPath = trajPath9;
    // }
    // else{
    //   trajPath = trajPath3;
    // }
    // Using file:
    // std::cout << "Using file: " << trajPath << std::endl;
    std::ifstream dataFile(trajPath);
    std::string line;
    int number_of_lines = 0;
    while (std::getline(dataFile, line)){
      number_of_lines++;
    }
    // printf("Number of lines: %d\n", number_of_lines);

    dataFile.clear();
    dataFile.seekg(0, std::ios::beg);

    std::uniform_int_distribution<int> initial_point(2, number_of_lines-501);

    int initial_point_index = initial_point(random_gen_);
    // int initial_point_index = 200;
      
    int current_line = 0;
    float initial_time = 0.0f;
    while (std::getline(dataFile, line)){
      current_line++;
      if (current_line == initial_point_index){
        std::istringstream iss(line); // Use string stream
        std::vector<std::string> data;
        std::string token;
        while (std::getline(iss, token, ',')) { // Use ',' as delimiter
          data.push_back(token);
        }
        initial_time = std::stof(data[0]);
        quad_state_.x(QS::POSX) = std::stof(data[1]);
        quad_state_.x(QS::POSY) = std::stof(data[2]);
        quad_state_.x(QS::POSZ) = std::stof(data[3]);
        quad_state_.x(QS::ATTW) = std::stof(data[4]);
        quad_state_.x(QS::ATTX) = std::stof(data[5]);
        quad_state_.x(QS::ATTY) = std::stof(data[6]);
        quad_state_.x(QS::ATTZ) = std::stof(data[7]);
        quad_state_.x(QS::VELX) = std::stof(data[8]);
        quad_state_.x(QS::VELY) = std::stof(data[9]);
        quad_state_.x(QS::VELZ) = std::stof(data[10]);

        // quad_state_.x(QS::OMEX) = std::stof(data[11]);
        // quad_state_.x(QS::OMEY) = std::stof(data[12]);
        // quad_state_.x(QS::OMEZ) = std::stof(data[13]);
        // quad_state_.x(QS::ACCX) = std::stof(data[14]);
        // quad_state_.x(QS::ACCY) = std::stof(data[15]);
        // quad_state_.x(QS::ACCZ) = std::stof(data[16]);
        // std::cout << "Initial Position: " << quad_state_.x.segment<quadenv::kNPos>(quadenv::kPos).transpose() << std::endl;

        // Add a small random noise to the initial state
        if (add_random_noise_temp){
          quad_state_.x(QS::POSX) += 0.1*uniform_dist_(random_gen_);
          quad_state_.x(QS::POSY) += 0.1*uniform_dist_(random_gen_);
          quad_state_.x(QS::POSZ) += 0.1*uniform_dist_(random_gen_);
          quad_state_.x(QS::VELX) += 0.1*uniform_dist_(random_gen_);
          quad_state_.x(QS::VELY) += 0.1*uniform_dist_(random_gen_);
          quad_state_.x(QS::VELZ) += 0.1*uniform_dist_(random_gen_);
        }
        // quad_state_.qx /= quad_state_.qx.norm();

        break;
      }
    }
    bool goal_set = false;
    while (std::getline(dataFile, line)) {
      std::istringstream iss(line);
      std::vector<std::string> data;
      std::string token;
      while (std::getline(iss, token, ',')) {
        data.push_back(token);
      }
      if (goal_set == false) {
        if (std::stof(data[0]) - initial_time > time_partition_window){
          // std::cout << "TIMES: " << initial_time << stof(data[0]) << std::endl;
          goal_state_(QS::POSX) = std::stof(data[1]);
          goal_state_(QS::POSY) = std::stof(data[2]);
          goal_state_(QS::POSZ) = std::stof(data[3]);
          goal_state_(QS::ATTW) = std::stof(data[4]);
          goal_state_(QS::ATTX) = std::stof(data[5]);
          goal_state_(QS::ATTY) = std::stof(data[6]);
          goal_state_(QS::ATTZ) = std::stof(data[7]);
          goal_state_(QS::VELX) = std::stof(data[8]);
          goal_state_(QS::VELY) = std::stof(data[9]);
          goal_state_(QS::VELZ) = std::stof(data[10]);
          goal_set = true;
          // goal_state_(QS::OMEX) = std::stof(data[11]);
          // goal_state_(QS::OMEY) = std::stof(data[12]);
          // goal_state_(QS::OMEZ) = std::stof(data[13]);
        }
      }
      else if (std::stof(data[0]) - initial_time > measurement_partition_window) {
        break;
      }
      // Push the state into the trajectory
      Vector<10> state;
      state << std::stof(data[1]), std::stof(data[2]), std::stof(data[3]), std::stof(data[4]), std::stof(data[5]), std::stof(data[6]), std::stof(data[7]), std::stof(data[8]), std::stof(data[9]), std::stof(data[10]);
      traj_.push_back(state);
    }
    // Length of traj_
    // std::cout << "Trajectory Length: " << traj_.size() << std::endl;

      // printf("Starting at time: %f\n", initial_time);
      // printf("Starting at position: %f, %f, %f\n", quad_state_.x(QS::POSX), quad_state_.x(QS::POSY), quad_state_.x(QS::POSZ));
      // Check next line for time_partition_window


      // while (std::getline(dataFile, line)){
      //   std::istringstream iss(line);
      //   std::vector<std::string> data;
      //   std::string token;
      //   while (std::getline(iss, token, ',')) {
      //     data.push_back(token);
      //   }
    // int next_point_distance = 0;
    // if (point_to_point)
    //   next_point_distance = 1;
    // else
    //   next_point_distance = 25;

    // // if (std::stof(data[0]) - initial_time > time_partition_window){
    // // If the current line is 15 lines away from the initial point, then we can use this as the goal state
    // for (int i = 0; i < next_point_distance; i++){
    //   std::getline(dataFile, line);
    //   std::istringstream iss(line);
    //   std::vector<std::string> data;
    //   std::string token;
    //   while (std::getline(iss, token, ',')) {
    //     data.push_back(token);
    //   }
    // }

    // Close file if necessary
    dataFile.close();
    // Print Drone's initial position and goal position
    // std::cout << "Goal Position: " << goal_state_.segment<quadenv::kNPos>(quadenv::kPos).transpose() << std::endl;
    // wait 0.1s
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  quadrotor_ptr_->reset(quad_state_);

  cmd_.t = 0.0;
  cmd_.thrusts.setZero();

  getObs(obs);
  return true;
}

bool QuadrotorEnvByDataProg::resetRange(Ref<Vector<>> obs, int lower_zbound, int upper_zbound, int lower_xybound, int upper_xybound, const bool random) {
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

// bool QuadrotorEnvByDataProg::getObs(Ref<Vector<>> obs) {
//   quadrotor_ptr_->getState(&quad_state_);

//   // convert quaternion to euler angle
//   Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
//   // quaternionToEuler(quad_state_.q(), euler);
//   quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w, goal_state_;

//   // obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
//   return true;
// }

bool QuadrotorEnvByDataProg::getObs(Ref<Vector<>> obs) {
  quadrotor_ptr_->getState(&quad_state_);

  // convert quaternion to euler angle
  Vector<3> euler_zyx = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
  // quaternionToEuler(quad_state_.q(), euler);
  quad_obs_ << quad_state_.p, euler_zyx, quad_state_.v, quad_state_.w;

  obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;

  if (prog_debug_observations){
    // Print Full Observation 
    std::cout << obs.transpose() << std::endl;
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  }

  return true;

  // obs.segment<quadenv::kNObs>(quadenv::kObs) = quad_obs_;
  //   obs.segment<quadenv::kNObs>(quadenv::kObs + quadenv::kNObs) = goal_state_;
  // obs(quadenv::kNObs) = goal_state_(QS::POSZ); // add goal state to observation vector

  // NEW APPROACH: We start with drone velocity and orientation
  // Then we add the relative position from the drone to the next 50 points in the trajectory
  // Drone Velocity and Orientation without position

  // Clear Observation Vector
  // obs.setZero();
  // obs.segment<3>(0) = quad_obs_.segment<3>(3);
  // obs.segment<3>(3) = quad_obs_.segment<3>(6); //TODO: Verify These

  // // Append Relative Position from Drone to Next 50 Points in Trajectory
  // // std::cout << "Trajectory Length: " << traj_.size() << std::endl;
  // // std::cout << "Current Step: " << mid_train_step_ << std::endl;
  // // std::cout << "Current Trajectory Point: " << traj_[mid_train_step_].transpose() << std::endl;
  // int skipped_points = 0;
  // for (int i = mid_train_step_; i < mid_train_step_ + 50; i++){
  //   // if (i%4 == 0) {
  //     if (i < traj_.size()){
  //       obs.segment<quadenv::kNPos>(quadenv::kPos + 6 + 3*(i-mid_train_step_-skipped_points)) = traj_[i].segment<3>(0) - quad_state_.x.segment<3>(0);
  //       // // Convert Orientation to Euler Angles
  //       // Vector<4> traj_quat = traj_[i].segment<4>(3);
  //       // Eigen::Quaternion<Scalar> traj_orientation(traj_quat(0), traj_quat(1), traj_quat(2), traj_quat(3));
  //       // Vector<3> traj_euler = traj_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
  //       // // Add Orientation to Observation
  //       // obs.segment<quadenv::kNPos>(quadenv::kPos + 6 + 6*(i-mid_train_step_-skipped_points)+3) = traj_euler;
  //     }
  //     else{
  //       obs.segment<quadenv::kNPos>(quadenv::kPos + 6 + 3*(i-mid_train_step_-skipped_points)) = Vector<3>::Zero();
  //     }
  //   // }
  //   // else {
  //   //   skipped_points++;
  //   // }
  // }

  // Print Full Drone State
  // std::cout << quad_obs_.transpose() << std::endl;
  // // Print Full Goal State
  // std::cout << goal_state_.transpose() << std::endl;

  // Print Full Observation 
  // std::cout << obs.transpose() << std::endl;
  // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

  // Print Obs dimension
  // std::cout << obs.size() << std::endl;

  return true;
}

Scalar QuadrotorEnvByDataProg::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {

  if (log_positions_to_files) {
    // Log the current position with a comma in between each value
    std::ofstream logFile;
    logFile.open("positions.csv", std::ios::app);
    logFile << quad_state_.x(0) << "," << quad_state_.x(1) << std::endl;
    logFile.close();
  }

  // Print the time and the step
  mid_train_step_++;

  if (prog_debug_time) {
  std::cout << "Taking Step: " << mid_train_step_ << std::endl;
  std::cout << "Time: " << cmd_.t << ", Step: " << mid_train_step_ << std::endl;
  }
  // // Print out the trajectory
  // for (int i = 0; i < traj_.size(); i++){
  //   std::cout << "Trajectory Point " << i << ": " << traj_[i].transpose() << std::endl;
  // }
  quad_act_ = act.cwiseProduct(act_std_) + act_mean_;
  // // Set quad_act to hover control, m*g/11.02,0,0,0
  // quad_act_ << 7.12, 0.0, 0.0, 0.0;    # CONCERNING
  // // Set quadrotor orientation to be 0,0,0,0
  // quad_state_.x.segment<4>(3) << 0.0, 0.0, 0.0, 0.0;
  // // Set velocity to be 0,0,0
  // quad_state_.x.segment<2>(6) << 0.0, 0.0;
  // quadrotor_ptr_->reset(quad_state_);
  // std::cout << "Act: " << quad_act_.transpose() << std::endl;

  cmd_.t += sim_dt_;
  cmd_.thrusts = quad_act_;

  // simulate quadrotor
  quadrotor_ptr_->run(cmd_, sim_dt_);

  // update observations
  getObs(obs);

  Matrix<3, 3> rot = quad_state_.q().toRotationMatrix();
  Scalar total_reward = 0.0;

  Scalar pos_reward = 0.0;
  Scalar vel_reward = 0.0;

  // New Reward Function, Trajectory Tracking Instead of comparing to goal state
  // print length of traj_
  // std::cout << "Trajectory Length: " << traj_.size() << std::endl;
  int trajectory_length = traj_.size();
  int trajectory_length_boundary = int(prog_train_horizon*100)-(prog_view_horizon*100);
  // if ((mid_train_step_*2)-1 < trajectory_length/2){
  if ((mid_train_step_*2)-1 < trajectory_length_boundary){
    int desired_pose_index = mid_train_step_*2 - 1;
    // Print current pose and desired pose
    // std::cout << "Current Pose: " << quad_state_.x.segment<10>(0).transpose() << std::endl;
    // std::cout << "Desired Pose: " << traj_[desired_pose_index].transpose() << std::endl;
    // Pos Reward, Ori Reward, Lin Vel Reward
    // for (int i = 1; i < 10; i++){
    //   // total_reward += (quad_state_(i) - desired_pose(i))*(quad_state_(i) - desired_pose(i)) * pos_coeff_;
    //   total_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_;
    //   // std::cout << (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_ << std::endl;
    //   // total_reward += 0.01;
    // }
    for (int i = 0; i < 3; i++){
      // total_reward += (quad_state_(i) - desired_pose(i))*(quad_state_(i) - desired_pose(i)) * pos_coeff_;
      total_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*10;
      pos_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*10;
      if (std::isnan((quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*10)){
        std::cout << "NAN" << std::endl;
      }
      // std::cout << (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_ << std::endl;
      // total_reward += 0.01;
    }
    // for (int i = 3; i < 7; i++) {
    //   total_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*10;
    //   if (std::isnan((quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*10)){
    //     std::cout << "NAN" << std::endl;
    //   }
    //   // std::cout << "Current Orientation: " << i << "is: " << quad_state_.x(i) << std::endl;
    //   // std::cout << "Desired Orientation: " << i << "is: " << traj_[desired_pose_index](i) << std::endl;
    // }
    
    // Get normalized orientation for current state and desired state, this is zyx euler angles
    // Vector<3> current_orientation = quad_state_.q().toRotationMatrix().eulerAngles(2, 1, 0);
    // // Vector<3> current_orientation;
    // // current_orientation << quad_state_.x(3), quad_state_.x(4), quad_state_.x(5);

    // // Traj Points format: px, py, pz, qw, qx, qy, qz, vx, vy, vz
    // Vector<3> desired_orientation = traj_[desired_pose_index].segment<3>(3);
    // // Convert desired orientation to euler angles, this is zyx euler angles
    // Eigen::Quaternion<Scalar> desired_orientation_quat(desired_orientation(0), desired_orientation(1), desired_orientation(2), desired_orientation(3));
    // desired_orientation = desired_orientation_quat.toRotationMatrix().eulerAngles(2, 1, 0);

    Vector<4> current_orientation = quad_state_.x.segment<4>(quadenv::kOri);
    Vector<4> desired_orientation = traj_[desired_pose_index].segment<4>(3);
    
    // std::cout << "Current Orientation: " << current_orientation.transpose() << std::endl;
    // std::cout << "Desired Orientation: " << desired_orientation.transpose() << std::endl;

    // Normalize the orientation -> not necessary for euler angles
    if (prog_enable_orientation_reward) {
    current_orientation /= current_orientation.norm();
    desired_orientation /= desired_orientation.norm();
    }


    // Apply reward function for orientation
    if (prog_enable_orientation_reward) {
      for (int i = 0; i < 4; i++){
        total_reward += (current_orientation(i) - desired_orientation(i))*(current_orientation(i) - desired_orientation(i)) * pos_coeff_*10;
        if (std::isnan((current_orientation(i) - desired_orientation(i))*(current_orientation(i) - desired_orientation(i)) * pos_coeff_*10)){
          std::cout << "NAN" << std::endl;
        }
      }
      if (prog_debug_orientations) {
        std::cout << "Current Orientation: " << current_orientation(0) << ", " << current_orientation(1) << ", " << current_orientation(2) << ", " << current_orientation(3) << std::endl;
        std::cout << "Desired Orientation: " << desired_orientation(0) << ", " << desired_orientation(1) << ", " << desired_orientation(2) << ", " << desired_orientation(3) << std::endl;
        std::cout << "Orientation Difference: " << current_orientation(0)-desired_orientation(0) << ", " << current_orientation(1)-desired_orientation(1) << ", " << current_orientation(2)-desired_orientation(2) << ", " << current_orientation(3)-desired_orientation(3) << std::endl;
        // Log Using Degrees, The returned angles are in the ranges [0:pi]x[-pi:pi]x[-pi:pi]
        // std::cout << "Current Orientation: " << current_orientation(0)*180/M_PI << ", " << current_orientation(1)*180/M_PI << ", " << current_orientation(2)*180/M_PI << std::endl;
        // std::cout << "Desired Orientation: " << desired_orientation(0)*180/M_PI << ", " << desired_orientation(1)*180/M_PI << ", " << desired_orientation(2)*180/M_PI << std::endl;
        // std::cout << "Orientation Difference: " << (current_orientation(0)-desired_orientation(0))*180/M_PI << ", " << (current_orientation(1)-desired_orientation(1))*180/M_PI << ", " << (current_orientation(2)-desired_orientation(2))*180/M_PI << std::endl;
      }
    }

    for (int i = 7; i < 10; i++){
      // total_reward += (quad_state_(i) - desired_pose(i))*(quad_state_(i) - desired_pose(i)) * pos_coeff_;
      total_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*5;
      vel_reward += (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*5;
      if (std::isnan((quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_*5)){
        std::cout << "NAN" << std::endl;
      }
      // std::cout << (quad_state_.x(i) - traj_[desired_pose_index](i))*(quad_state_.x(i) - traj_[desired_pose_index](i)) * pos_coeff_ << std::endl;
      // total_reward += 0.01;
    }
    // total_reward += (quad_state_.x(0) - traj_[desired_pose_index](0))*(quad_state_.x(0) - traj_[desired_pose_index](0)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(1) - traj_[desired_pose_index](1))*(quad_state_.x(1) - traj_[desired_pose_index](1)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(2) - traj_[desired_pose_index](2))*(quad_state_.x(2) - traj_[desired_pose_index](2)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(3) - traj_[desired_pose_index](3))*(quad_state_.x(3) - traj_[desired_pose_index](3)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(4) - traj_[desired_pose_index](4))*(quad_state_.x(4) - traj_[desired_pose_index](4)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(5) - traj_[desired_pose_index](5))*(quad_state_.x(5) - traj_[desired_pose_index](5)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(6) - traj_[desired_pose_index](6))*(quad_state_.x(6) - traj_[desired_pose_index](6)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(7) - traj_[desired_pose_index](7))*(quad_state_.x(7) - traj_[desired_pose_index](7)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(8) - traj_[desired_pose_index](8))*(quad_state_.x(8) - traj_[desired_pose_index](8)) * pos_coeff_ /10;
    // total_reward += (quad_state_.x(9) - traj_[desired_pose_index](9))*(quad_state_.x(9) - traj_[desired_pose_index](9)) * pos_coeff_ /10;

    // Clamp the reward to be between -5 and 5
    if (total_reward > 3){
      total_reward = 3;
    }
    else if (total_reward < -3){
      total_reward = -3;
    }
    // std::cout << "Total Reward: " << total_reward << std::endl;
    // std::cout << "Current Position X: " << quad_state_.x(0) << std::endl;
    // std::cout << "Desired Position X: " << traj_[desired_pose_index](0) << std::endl;
    // total_reward += (quad_state_x(1) - traj_[desired_pose_index](1))*(quad_state_x(1) - traj_[desired_pose_index](1)) * pos_coeff_;
  // Scalar pos_reward =
  //   pos_coeff_ * (quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) -
  //                 goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
  //                  .squaredNorm();
  // // - orientation tracking
  // Scalar ori_reward =
  //   ori_coeff_ * (quad_obs_.segment<quadenv::kNOri>(quadenv::kOri) -
  //                 goal_state_.segment<quadenv::kNOri>(quadenv::kOri))
  //                  .squaredNorm();
  // // - linear velocity tracking
  // Scalar lin_vel_reward =
  //   lin_vel_coeff_ * (quad_obs_.segment<quadenv::kNLinVel>(quadenv::kLinVel) -
  //                     goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel))
  //                      .squaredNorm();

  // // - angular velocity tracking
  // Scalar ang_vel_reward =
  //   ang_vel_coeff_ * (quad_obs_.segment<quadenv::kNAngVel>(quadenv::kAngVel) -
  //                     goal_state_.segment<quadenv::kNAngVel>(quadenv::kAngVel))
  //                      .squaredNorm();
  //   // survival reward
  //   total_reward += 0.3 + pos_reward + ori_reward + lin_vel_reward + ang_vel_reward;
    // total_reward += 0.01;
    // Check if total_reward is NaN
    if (std::isnan(static_cast<double>(total_reward))){
      total_reward = 0;
      std::cout << "Total Reward is NaN" << std::endl;
    }
    if (std::isinf(static_cast<double>(total_reward))){
      total_reward = 0;
      std::cout << "Total Reward is Inf" << std::endl;
    }
    // Print Current Time, Step, and trajectory length
    if (prog_debug_horizons) {
      std::cout << "Time: " << cmd_.t << ", Step: " << mid_train_step_ << ", Trajectory Length: " << trajectory_length << std::endl;
    }
    if (prog_debug_velocities) {
      // Display Drone's Velocity Magnitude
      std::cout << "Drone's Velocity: " << quad_state_.x.segment<quadenv::kNLinVel>(quadenv::kLinVel).norm() << std::endl;
      // Display Goal Velocity Magnitude
      Vector<3> target_velocity_;
      target_velocity_ = traj_[desired_pose_index].segment<3>(7);
      std::cout << "Goal Velocity: " << target_velocity_.norm() << std::endl;
    }

    // - control action penalty
    Vector<4> hover_action;
    hover_action << 9.8/4, 9.8/4, 9.8/4, 9.8/4;
    // act_reward is the L2 norm of the difference between the current action and the hover action
    Scalar act_reward = 10 * act_coeff_ * (quad_act_ - hover_action).norm();
    total_reward += act_reward;

    if (prog_debug_actions) {
      // Print out the action vector
      std::cout << "Action Vector: " << quad_act_.transpose() << std::endl;
      // Print out the action reward
      std::cout << "Action Reward: " << act_reward << std::endl;
    }
    
    if (prog_debug_total_reward) {
    std::cout << "Total Reward: " << total_reward << std::endl;
    std::cout << "Action Reward: " << act_reward << std::endl;
    std::cout << "Position Reward: " << pos_reward << std::endl;
    std::cout << "Velocity Reward: " << vel_reward << std::endl;
    }

    return total_reward;
  }
  else {
    // Punish the drone for not reaching the goal state
    return -0.3;
  }
}

bool QuadrotorEnvByDataProg::isTerminalState(Scalar &reward) {
  // if ((((quad_state_.x.segment<quadenv::kNPos>(quadenv::kPos) -
  //      goal_state_.segment<quadenv::kNPos>(quadenv::kPos))
  //       .squaredNorm() < 0.1))) {
  //       // return false;
  //   // We want the quadrotor to terminate within 0.1m of the goal, and reward it immediately for doing so
  //   // double dist = (quad_obs_.segment<quadenv::kNPos>(quadenv::kPos) - goal_state_.segment<quadenv::kNPos>(quadenv::kPos)).squaredNorm();
  //   // double power = -0.5*std::pow(dist/0.5, 2);
  //   // reward = 10.0*std::exp(power);
  //   reward = 30.0;
  //   // reward = 0;

  //   // // Use a bell curve to reward the drone for having a velocity that is very close to the desired velocity
  //   // // MAXIMUM REWARD FROM VELOCITY: 40.0, MINIMUM REWARD FROM VELOCITY: 0.0
  //   double vel_dist = (quad_state_.x.segment<quadenv::kNLinVel>(quadenv::kLinVel) - goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel)).squaredNorm();
  //   double vel_power = -0.5*std::pow(vel_dist/0.5, 2);
  //   // reward += 40.0*std::exp(vel_power);


  //   // Use a bell curve to reward the drone for having a terminal orientation that is very close to the desired orientation
  //   // MAXIMUM REWARD FROM ORIENTATION: 40.0, MINIMUM REWARD FROM ORIENTATION: 0.0
  //   double ori_dist = (quad_state_.x.segment<quadenv::kNOri>(quadenv::kOri) - goal_state_.segment<quadenv::kNOri>(quadenv::kOri)).squaredNorm();
  //   double ori_power = -0.5*std::pow(ori_dist/0.5, 2);
  //   // reward += 40.0*std::exp(ori_power);
  //   // also print the distances
  //   // Display Drone's velocity and goal velocity
  //   // std::cout << "Drone's Velocity: " << quad_state_.x.segment<quadenv::kNLinVel>(quadenv::kLinVel).transpose() << std::endl;
  //   // std::cout << "Goal Velocity: " << goal_state_.segment<quadenv::kNLinVel>(quadenv::kLinVel).transpose() << std::endl;
  //   std::cout << "Orientation diff: " << ori_dist << std::endl;
  //   // Display Velocity and Orientation Rewards
  //   // std::cout << "Velocity Reward: " << 50.0*std::exp(vel_power) << std::endl;
  //   std::cout << "Velocity diff: " << vel_dist << std::endl;
  //   // std::cout << "Orientation Reward: " << 50.0*std::exp(ori_power) << std::endl;
  //   // std::cout << "Terminal Reward: " << reward << std::endl;
  //   return true;
  // }
  // else if ((quad_state_.x(QS::POSZ) <= -10.0)) {
  //   reward = -10.5;
  //   return true;
  // }
  // Once mid__
  int trajectory_length = traj_.size();
  // if ((mid_train_step_*2)-1 >= trajectory_length/2) { //Slightly off, should be a bit smaller
  int trajectory_length_boundary = int(prog_train_horizon*100)-(prog_view_horizon*100);
  if ((mid_train_step_*2)-1 >= trajectory_length_boundary) { //Slightly off, should be a bit smaller
    if (prog_debug_horizons){
      std::cout << "Reached End of Trajectory" << std::endl;
      std::cout << "Mid Train Step: " << mid_train_step_ << std::endl;
    }
    // std::cout << "Full Trial Reward" << reward << std::endl;
    // reward = 5;
    return true;
  }
  else {
    reward = 0.0;
    return false;
  }

}

bool QuadrotorEnvByDataProg::loadParam(const YAML::Node &cfg) {
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

bool QuadrotorEnvByDataProg::getAct(Ref<Vector<>> act) const {
  if (cmd_.t >= 0.0 && quad_act_.allFinite()) {
    act = quad_act_;
    return true;
  }
  return false;
}

bool QuadrotorEnvByDataProg::getAct(Command *const cmd) const {
  if (!cmd_.valid()) return false;
  *cmd = cmd_;
  return true;
}

void QuadrotorEnvByDataProg::addObjectsToUnity(std::shared_ptr<UnityBridge> bridge) {
  bridge->addQuadrotor(quadrotor_ptr_);
}

std::ostream &operator<<(std::ostream &os, const QuadrotorEnvByDataProg &quad_env) {
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