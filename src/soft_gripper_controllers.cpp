#include "soft_gripper_controllers/soft_gripper_controllers.h"

const uint8_t ADDR_GOAL_CURRENT = 102;
const uint8_t ADDR_GOAL_PWM = 100;
const uint8_t LEN_GOAL_CURRENT = 2;
const uint8_t LEN_GIAL_PWM = 2;

int cycle_index;

DynamixelController::DynamixelController()
  :node_handle_(""),
   priv_node_handle_("~"),
   is_joint_state_topic_(false),
   is_cmd_vel_topic_(false),
   use_moveit_(false),
   wheel_separation_(0.0f),
   wheel_radius_(0.0f),
   is_moving_(false)
{
  is_joint_state_topic_ = priv_node_handle_.param<bool>("use_joint_states_topic", true);
  is_cmd_vel_topic_ = priv_node_handle_.param<bool>("use_cmd_vel_topic", false);
  use_moveit_ = priv_node_handle_.param<bool>("use_moveit", false);

  read_period_ = priv_node_handle_.param<double>("dxl_read_period", 0.010f);
  write_period_ = priv_node_handle_.param<double>("dxl_write_period", 0.010f);
  pub_period_ = priv_node_handle_.param<double>("publish_period", 0.010f);

  if (is_cmd_vel_topic_ == true)
  {
    wheel_separation_ = priv_node_handle_.param<double>("mobile_robot_config/seperation_between_wheels", 0.0);
    wheel_radius_ = priv_node_handle_.param<double>("mobile_robot_config/radius_of_wheel", 0.0);
  }

  dxl_wb_ = new DynamixelWorkbench;
  jnt_tra_ = new JointTrajectory;
  jnt_tra_msg_ = new trajectory_msgs::JointTrajectory;
}

DynamixelController::~DynamixelController(){}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    dxl_wb_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool DynamixelController::gripper_torqueoff(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);
  }

  return true;
}


bool DynamixelController::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  //Add goal_current ControlItem
  const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
  if (goal_current== NULL)  return false;

  const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
  if (goal_position == NULL) return false;

  const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
  if (goal_velocity == NULL)  goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
  if (goal_velocity == NULL)  return false;

  const ControlItem *goal_pwm = dxl_wb_->getItemInfo(it->second, "Goal_PWM");
  if (goal_pwm == NULL)  return false;

  const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
  if (present_position == NULL) return false;

  const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
  if (present_velocity == NULL)  present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
  if (present_velocity == NULL) return false;

  const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
  if (present_current == NULL)  present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
  if (present_current == NULL) return false;

  control_items_["Goal_Position"] = goal_position;
  control_items_["Goal_Velocity"] = goal_velocity;
  //Add "Goal_Current"
  control_items_["Goal_Current"] = goal_current;
  control_items_["Goal_PWM"] = goal_pwm;

  control_items_["Present_Position"] = present_position;
  control_items_["Present_Velocity"] = present_velocity;
  control_items_["Present_Current"] = present_current;

  return true;
}

// void DynamixelController::registerControlInterfaces()
//   {
//     // resize vector
//     uint8_t joint_size = 0;
//     for (auto const &dxl : dynamixel_)
//     {
//       if (joint_size < (uint8_t)dxl.second)
//         joint_size = (uint8_t)dxl.second;
//     }
//     joints_.resize(joint_size);
//
//     for (auto iter = dynamixel_.begin(); iter != dynamixel_.end(); iter++)
//     {
//       // initialize joint vector
//       Joint joint;
//       joints_[iter->second - 1] = joint;
//       ROS_INFO("joint_name : %s, servo ID: %d", iter->first.c_str(), iter->second);
//
//       // connect and register the joint state interface
//       hardware_interface::JointStateHandle joint_state_handle(iter->first.c_str(),
//                                                               &joints_[iter->second - 1].position,
//                                                               &joints_[iter->second - 1].velocity,
//                                                               &joints_[iter->second - 1].effort);
//       joint_state_interface_.registerHandle(joint_state_handle);
//
//       // connect and register the joint position, velocity and effort interface
//       hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[iter->second - 1].position_command);
//       position_joint_interface_.registerHandle(position_joint_handle);
//       hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &joints_[iter->second - 1].velocity_command);
//       velocity_joint_interface_.registerHandle(velocity_joint_handle);
//       hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &joints_[iter->second - 1].effort_command);
//       effort_joint_interface_.registerHandle(effort_joint_handle);
//     }
//
//     registerInterface(&joint_state_interface_);
//     registerInterface(&position_joint_interface_);
//     registerInterface(&velocity_joint_interface_);
//     registerInterface(&effort_joint_interface_);
//   }

bool DynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("%s", log);
  // }

  // result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
  // if (result == false)
  // {
  //   ROS_ERROR("%s", log);
  //   return result;
  // }
  // else
  // {
  //   ROS_INFO("%s", log);
  // }

  //add goal_current addSyncWriteHandler
  //result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
  result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_PWM"]->address, control_items_["Goal_PWM"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

    /*
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length+2;

    result = dxl_wb_->addSyncReadHandler(start_address,
                                          read_length,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }

    // result = dxl_wb_->addSyncWriteHandler(ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, &log);
    // if (result == false)
    // {
    //   ROS_ERROR("%s", log);
    //   return result;
    // }
  }

  return result;
}

bool DynamixelController::gripper_read()
{
  bool result = false;
  const char* log = NULL;

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];
  uint8_t id_array[dynamixel_.size()];

  int id_cnt = 0;
  for (auto const& dxl:dynamixel_)
  {
      // ROS_INFO_STREAM("joints_ id: " << (int)dxl.second << " pos cmd: " << (joints_[(uint8_t)dxl.second - 1].position_command));
      id_array[id_cnt] = (uint8_t)dxl.second;
      id_cnt++;
  }
  uint8_t id_array_g[1] = {id_array[0]};

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    WayPoint wp;

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Current"]->address,
                                                  control_items_["Present_Current"]->data_length,
                                                  get_current,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }else
    {
      for(uint8_t index = 0; index < id_cnt; index++)
      {
        jaw_current.push_back(dxl_wb_->convertValue2Current(id_array[index], get_current[index]));
        ROS_INFO("jaw current: %f", jaw_current[cycle_index]);
      }
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Velocity"]->address,
                                                  control_items_["Present_Velocity"]->data_length,
                                                  get_velocity,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }else
    {
      for(uint8_t index = 0; index < id_cnt; index++)
      {
        jaw_velocity.push_back(dxl_wb_->convertValue2Velocity(id_array[index], get_velocity[index]));
        ROS_INFO("jaw velocity: %f", jaw_velocity[cycle_index]);
      }
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  id_array,
                                                  id_cnt,
                                                  control_items_["Present_Position"]->address,
                                                  control_items_["Present_Position"]->data_length,
                                                  get_position,
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }else
    {
      for(uint8_t index = 0; index < id_cnt; index++)
      {
        jaw_position.push_back( (dxl_wb_->convertValue2Radian(id_array[index], get_position[index]))/3.14*180/0.088 );
        ROS_INFO("jaw position: %f", jaw_position[cycle_index]);
      }
    }

    // result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
    //                                   id_array,
    //                                   id_cnt,
    //                                   control_items_["Present_Position"]->address,
    //                                   control_items_["Present_Position"]->data_length,
    //                                   get_position,
    //                                   &log);
    //
    // if (result == false)
    // {
    //   ROS_ERROR("%s", log);
    // }
    // else
    // {
    //   for(uint8_t index = 0; index < id_cnt; index++)
    //   {
    //     jaw_position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
    //     ROS_INFO("jaw position: %f", jaw_position);
    //     wp.position = dxl_wb_->convertValue2Radian(id_array[index], get_position[index]);
    //     wp.velocity = 0.0f;
    //     wp.acceleration = 0.0f;
    //     pre_goal_.push_back(wp);
    //   }
    // }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    WayPoint wp;
    uint32_t read_position;
    for (auto const& dxl:dynamixel_)
    {
      result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                     control_items_["Present_Position"]->address,
                                     control_items_["Present_Position"]->data_length,
                                     &read_position,
                                     &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      wp.position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, read_position);
      wp.velocity = 0.0f;
      wp.acceleration = 0.0f;
      pre_goal_.push_back(wp);
    }
  }

  return result;
}

void DynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  if (is_joint_state_topic_) joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber()
{
  trajectory_sub_ = priv_node_handle_.subscribe("joint_trajectory", 100, &DynamixelController::trajectoryMsgCallback, this);
  if (is_cmd_vel_topic_) cmd_vel_sub_ = priv_node_handle_.subscribe("cmd_vel", 10, &DynamixelController::commandVelocityCallback, this);
}

void DynamixelController::initServer()
{
  dynamixel_command_server_ = priv_node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::readCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_read_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_current[dynamixel_.size()];
  int32_t get_velocity[dynamixel_.size()];
  int32_t get_position[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }
#ifndef DEBUG
  if (is_moving_ == false)
  {
#endif
    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                  id_array,
                                  dynamixel_.size(),
                                  &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    id_cnt,
                                                    control_items_["Present_Current"]->address,
                                                    control_items_["Present_Current"]->data_length,
                                                    get_current,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    id_cnt,
                                                    control_items_["Present_Velocity"]->address,
                                                    control_items_["Present_Velocity"]->data_length,
                                                    get_velocity,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                    id_array,
                                                    id_cnt,
                                                    control_items_["Present_Position"]->address,
                                                    control_items_["Present_Position"]->data_length,
                                                    get_position,
                                                    &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
      }

      for(uint8_t index = 0; index < id_cnt; index++)
      {
        dynamixel_state[index].present_current = get_current[index];
        dynamixel_state[index].present_velocity = get_velocity[index];
        dynamixel_state[index].present_position = get_position[index];
        // joints_[id_array[index]-1].effort = dxl_wb_->convertValue2Current((int16_t)get_current[index]);

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[index]);
        // joints_[id_array[index]- 1].effort_command = joints_[id_array[index] - 1].effort;
      }
    }
    else if(dxl_wb_->getProtocolVersion() == 1.0f)
    {
      uint16_t length_of_data = control_items_["Present_Position"]->data_length +
                                control_items_["Present_Velocity"]->data_length +
                                control_items_["Present_Current"]->data_length;
      uint32_t get_all_data[length_of_data];
      uint8_t dxl_cnt = 0;
      for (auto const& dxl:dynamixel_)
      {
        result = dxl_wb_->readRegister((uint8_t)dxl.second,
                                       control_items_["Present_Position"]->address,
                                       length_of_data,
                                       get_all_data,
                                       &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
        }

        dynamixel_state[dxl_cnt].present_current = DXL_MAKEWORD(get_all_data[4], get_all_data[5]);
        dynamixel_state[dxl_cnt].present_velocity = DXL_MAKEWORD(get_all_data[2], get_all_data[3]);
        dynamixel_state[dxl_cnt].present_position = DXL_MAKEWORD(get_all_data[0], get_all_data[1]);

        dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[dxl_cnt]);
        dxl_cnt++;
      }
    }
#ifndef DEBUG
  }
#endif

#ifdef DEBUG
  ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
  priv_read_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::gripper_write(double dxl_current)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  int32_t dynamixel_current[dynamixel_.size()];

  int cnt = 0;
  for (auto const& dxl:dynamixel_)
  {
      // ROS_INFO_STREAM("joints_ id: " << (int)dxl.second << " pos cmd: " << (joints_[(uint8_t)dxl.second - 1].position_command));
      id_array[cnt] = (uint8_t)dxl.second;
      dynamixel_current[cnt] =  dxl_wb_->convertCurrent2Value(dxl_current*2.69);
      cnt++;
  }

  // ROS_INFO_STREAM("id_array: " << int(id_array[0]));
  // ROS_INFO_STREAM("dynamixel_current: " << dynamixel_current[0]);

  uint8_t id_array_g[1] = {id_array[0]};
  int32_t dynamixel_current_g[1] = {dynamixel_current[0]};
  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT,id_array_g, cnt, dynamixel_current_g, 1, &log);
  // result = dxl_wb_->itemWrite(171, "Goal_Current", 5, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void DynamixelController::publishCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  dynamixel_state_list_pub_.publish(dynamixel_state_list_);

  if (is_joint_state_topic_)
  {
    joint_state_msg_.header.stamp = ros::Time::now();

    joint_state_msg_.name.clear();
    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;

      joint_state_msg_.name.push_back(dxl.first);

      if (dxl_wb_->getProtocolVersion() == 2.0f)
      {
        if (strcmp(dxl_wb_->getModelName((uint8_t)dxl.second), "XL-320") == 0) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
        else  effort = dxl_wb_->convertValue2Current((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      }
      else if (dxl_wb_->getProtocolVersion() == 1.0f) effort = dxl_wb_->convertValue2Load((int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);

      velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
      position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

      joint_state_msg_.effort.push_back(effort);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_.publish(joint_state_msg_);
  }

#ifdef DEBUG
  ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  double robot_lin_vel = msg->linear.x;
  double robot_ang_vel = msg->angular.z;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

//  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
//       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

  double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    if (strcmp(dxl_wb_->getModelName(id_array[0]), "XL-320") == 0)
    {
      if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
      else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
      else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

      if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
      else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
      else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
    }
    else
    {
      dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;
      dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;
    }
  }
  else if (dxl_wb_->getProtocolVersion() == 1.0f)
  {
    if (wheel_velocity[LEFT] == 0.0f) dynamixel_velocity[LEFT] = 0;
    else if (wheel_velocity[LEFT] < 0.0f) dynamixel_velocity[LEFT] = ((-1.0f) * wheel_velocity[LEFT]) * velocity_constant_value + 1023;
    else if (wheel_velocity[LEFT] > 0.0f) dynamixel_velocity[LEFT] = (wheel_velocity[LEFT] * velocity_constant_value);

    if (wheel_velocity[RIGHT] == 0.0f) dynamixel_velocity[RIGHT] = 0;
    else if (wheel_velocity[RIGHT] < 0.0f)  dynamixel_velocity[RIGHT] = ((-1.0f) * wheel_velocity[RIGHT] * velocity_constant_value) + 1023;
    else if (wheel_velocity[RIGHT] > 0.0f)  dynamixel_velocity[RIGHT] = (wheel_velocity[RIGHT] * velocity_constant_value);
  }

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void DynamixelController::writeCallback(const ros::TimerEvent&)
{
#ifdef DEBUG
  static double priv_pub_secs =ros::Time::now().toSec();
#endif
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  int32_t dynamixel_position[dynamixel_.size()];
  int32_t dynamixel_effort[dynamixel_.size()];

  static uint32_t point_cnt = 0;
  static uint32_t position_cnt = 0;

  for (auto const& joint:jnt_tra_msg_->joint_names)
  {
    id_array[id_cnt] = (uint8_t)dynamixel_[joint];
    id_cnt++;
  }
  // for (auto const &dxl : dynamixel_)
  //     {
  //       id_array[id_cnt] = (uint8_t)dxl.second;
  //       dynamixel_effort[id_cnt] = dxl_wb_->convertCurrent2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].effort / (1.78e-03));
  //       id_cnt++;
  //     }


  if (is_moving_ == true)
  {
    for (uint8_t index = 0; index < id_cnt; index++)
      dynamixel_position[index] = dxl_wb_->convertRadian2Value(id_array[index], jnt_tra_msg_->points[point_cnt].positions.at(index));
      //result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT, id_array, id_cnt, dynamixel_effort, 1, &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    position_cnt++;
    if (position_cnt >= jnt_tra_msg_->points[point_cnt].positions.size())
    {
      point_cnt++;
      position_cnt = 0;
      if (point_cnt >= jnt_tra_msg_->points.size())
      {
        is_moving_ = false;
        point_cnt = 0;
        position_cnt = 0;

        ROS_INFO("Complete Execution");
      }
    }
  }

#ifdef DEBUG
  ROS_WARN("[writeCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
  priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  uint8_t id_cnt = 0;
  bool result = false;
  WayPoint wp;

  if (is_moving_ == false)
  {
    jnt_tra_msg_->joint_names.clear();
    jnt_tra_msg_->points.clear();
    pre_goal_.clear();

    // result = getPresentPosition(msg->joint_names);
    // if (result == false)
    //   ROS_ERROR("Failed to get Present Position");
    //
    // for (auto const& joint:msg->joint_names)
    // {
    //   ROS_INFO("'%s' is ready to move", joint.c_str());
    //
    //   jnt_tra_msg_->joint_names.push_back(joint);
    //   id_cnt++;
    // }

    if (id_cnt != 0)
    {
      uint8_t cnt = 0;
      while(cnt < msg->points.size())
      {
        std::vector<WayPoint> goal;
        for (std::vector<int>::size_type id_num = 0; id_num < msg->points[cnt].positions.size(); id_num++)
        {
          wp.position = msg->points[cnt].positions.at(id_num);

          if (msg->points[cnt].velocities.size() != 0)  wp.velocity = msg->points[cnt].velocities.at(id_num);
          else wp.velocity = 0.0f;

          if (msg->points[cnt].accelerations.size() != 0)  wp.acceleration = msg->points[cnt].accelerations.at(id_num);
          else wp.acceleration = 0.0f;

          goal.push_back(wp);
        }

        if (use_moveit_ == true)
        {
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
          {
            jnt_tra_point_msg.positions.push_back(goal[id_num].position);
            jnt_tra_point_msg.velocities.push_back(goal[id_num].velocity);
            jnt_tra_point_msg.accelerations.push_back(goal[id_num].acceleration);
          }

          jnt_tra_msg_->points.push_back(jnt_tra_point_msg);

          cnt++;
        }
        else
        {
          jnt_tra_->setJointNum((uint8_t)msg->points[cnt].positions.size());

          double move_time = 0.0f;
          if (cnt == 0) move_time = msg->points[cnt].time_from_start.toSec();
          else move_time = msg->points[cnt].time_from_start.toSec() - msg->points[cnt-1].time_from_start.toSec();

          jnt_tra_->init(move_time,
                         write_period_,
                         pre_goal_,
                         goal);

          std::vector<WayPoint> way_point;
          trajectory_msgs::JointTrajectoryPoint jnt_tra_point_msg;

          for (double index = 0.0; index < move_time; index = index + write_period_)
          {
            way_point = jnt_tra_->getJointWayPoint(index);

            for (uint8_t id_num = 0; id_num < id_cnt; id_num++)
            {
              jnt_tra_point_msg.positions.push_back(way_point[id_num].position);
              jnt_tra_point_msg.velocities.push_back(way_point[id_num].velocity);
              jnt_tra_point_msg.accelerations.push_back(way_point[id_num].acceleration);
            }

            jnt_tra_msg_->points.push_back(jnt_tra_point_msg);
            jnt_tra_point_msg.positions.clear();
            jnt_tra_point_msg.velocities.clear();
            jnt_tra_point_msg.accelerations.clear();
          }

          pre_goal_ = goal;
          cnt++;
        }
      }
      ROS_INFO("Succeeded to get joint trajectory!");
      is_moving_ = true;
    }
    else
    {
      ROS_WARN("Please check joint_name");
    }
  }
  else
  {
    ROS_INFO_THROTTLE(1, "Dynamixel is moving");
  }
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

double DynamixelController::movingAvg(double a, double b, double beta)
{
  return beta*a + (1-beta)*b;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_workbench_controllers");
  ros::NodeHandle node_handle("");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 1000000;

  if (argc < 2)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  DynamixelController dynamixel_controller;

  bool result = false;

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  result = dynamixel_controller.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dynamixel_controller.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dynamixel_controller.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dynamixel_controller.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();

  gripper_data_pub = node_handle.advertise<sensor_msgs::ChannelFloat32>("/softgripper/gripper_states", 1);

  #define PI acos(-1)
  cycle_index = 0;
  double time_unit = 0.001;
  double reference_velocity = 1.8;

  std::vector<double> e_I;
  double k_p = 1;
  double k_i = 30;

  std::vector<double> k_ada;
  std::vector<double> l_ada;

  k_ada.push_back(4034.6);
  l_ada.push_back(40.345);
  
  int ita_1 = 1200;
  int ita_2 = 1500;
  
  int stop_count = 0;
  // double total_error = 0.0;
  
  gripper_states_data.values.resize(7);
  ros::Rate r(0.2);
  r.sleep();
  while (1) // approach phase
  {
    // reference_velocity = 1.5 + 0.1 * sin(time_unit * 2 * PI * cycle_index);
    dynamixel_controller.gripper_read();
    if (dynamixel_controller.jaw_velocity[cycle_index] > 100)
    {
      dynamixel_controller.jaw_velocity[cycle_index] = 0;
    }
    if (dynamixel_controller.jaw_current[cycle_index] > 1000)
    {
      dynamixel_controller.jaw_current[cycle_index] = 0;
    }

    dynamixel_controller.velocity_error.push_back(reference_velocity - dynamixel_controller.jaw_velocity[cycle_index]);

    if (cycle_index > 0)
    {
      // k_ada.push_back(-ita_1 * dynamixel_controller.velocity_error[cycle_index] * dynamixel_controller.jaw_velocity[cycle_index] * time_unit + k_ada[cycle_index - 1]);
      // l_ada.push_back( ita_2 * dynamixel_controller.velocity_error[cycle_index] * reference_velocity * time_unit + l_ada[cycle_index -1]);
      // dynamixel_controller.dxl_goal_current = -k_ada[cycle_index] * dynamixel_controller.jaw_velocity[cycle_index] + l_ada[cycle_index] * reference_velocity;
      
      // if (dynamixel_controller.dxl_goal_current < 0)
      // {
      //   dynamixel_controller.dxl_goal_current = 0;
      //   k_ada[cycle_index] = k_ada[cycle_index - 1];
      //   l_ada[cycle_index] = l_ada[cycle_index - 1];
      // }
      e_I.push_back( e_I[cycle_index-1]+(dynamixel_controller.velocity_error[cycle_index]-dynamixel_controller.velocity_error[cycle_index-1])*time_unit );
    }
    else
    {
      // dynamixel_controller.dxl_goal_current = -k_ada[cycle_index] * dynamixel_controller.jaw_velocity[cycle_index] + l_ada[cycle_index] * reference_velocity;
      e_I.push_back( 0.0+(dynamixel_controller.velocity_error[cycle_index]-0.0)*time_unit );
    }
    dynamixel_controller.dxl_goal_current = k_p*dynamixel_controller.velocity_error[cycle_index] + k_i*e_I[cycle_index];
    if (dynamixel_controller.dxl_goal_current > 8)
    {
      dynamixel_controller.dxl_goal_current = 8;
    }
    // if (dynamixel_controller.dxl_goal_current < 0)
    //   {
    //     dynamixel_controller.dxl_goal_current = 0;
    //   }
    dynamixel_controller.gripper_write(dynamixel_controller.dxl_goal_current*20);
    ROS_INFO("cycle_index: %d",cycle_index);
    ROS_INFO("result: %f",dynamixel_controller.dxl_goal_current);
    // ROS_INFO("velocity: %f",(double)((int)(dynamixel_controller.jaw_velocity[cycle_index]*1000))/1000.0);
    // ROS_INFO("stop_count: %d",stop_count);
    gripper_states_data.values[0] = static_cast<float>(dynamixel_controller.jaw_current[cycle_index]);
    gripper_states_data.values[1] = static_cast<float>(dynamixel_controller.jaw_position[cycle_index]);
    gripper_states_data.values[2] = static_cast<float>(dynamixel_controller.jaw_velocity[cycle_index]);
    gripper_states_data.values[3] = static_cast<float>(dynamixel_controller.dxl_goal_current);
    gripper_states_data.values[4] = static_cast<float>(dynamixel_controller.velocity_error[cycle_index]);
    gripper_states_data.values[5] = static_cast<float>(k_ada[cycle_index]);
    gripper_states_data.values[6] = static_cast<float>(l_ada[cycle_index]);
    gripper_data_pub.publish(gripper_states_data);
    if ( (((double)((int)(dynamixel_controller.jaw_velocity[cycle_index]*1000))/1000.0) <= 0.001) && cycle_index > 500)
    {
      stop_count = stop_count + 1;
    }else
    {
      stop_count = 0;
    }
    if (stop_count > 5)
    {
      break;
    }

    // if (dynamixel_controller.jaw_position[cycle_index] > 2048.0)
    // {
    //   break;
    // }
    
  
  cycle_index++;
  }

  int grasp_cycle_index = 0;
  while (1) //grasp phase
  { ROS_INFO("cycle_index: %d",cycle_index);
    ROS_INFO("grasp_cycle_index: %d",grasp_cycle_index);
    dynamixel_controller.dxl_goal_current = 300*(grasp_cycle_index*0.0003);
    dynamixel_controller.gripper_write(dynamixel_controller.dxl_goal_current);
    dynamixel_controller.gripper_read();
    gripper_states_data.values[0] = static_cast<float>(dynamixel_controller.jaw_current[cycle_index + grasp_cycle_index + 1]);
    gripper_states_data.values[1] = static_cast<float>(dynamixel_controller.jaw_position[cycle_index + grasp_cycle_index + 1]);
    gripper_states_data.values[2] = static_cast<float>(dynamixel_controller.jaw_velocity[cycle_index + grasp_cycle_index + 1]);
    gripper_states_data.values[3] = static_cast<float>(dynamixel_controller.dxl_goal_current);
    gripper_states_data.values[4] = static_cast<float>(dynamixel_controller.velocity_error[cycle_index + grasp_cycle_index + 1]);
    gripper_data_pub.publish(gripper_states_data);
    // filter after sending raw data. 
    
    double stop_detect = dynamixel_controller.jaw_position[cycle_index + grasp_cycle_index + 1]/dynamixel_controller.jaw_velocity[cycle_index + grasp_cycle_index + 1] * 0.00145 / ((cycle_index + grasp_cycle_index + 1) * time_unit);
    // dynamixel_controller.jaw_position[cycle_index + grasp_cycle_index + 1]/dynamixel_controller.jaw_velocity[cycle_index + grasp_cycle_index + 1] * 0.00145 / ((cycle_index + grasp_cycle_index + 1) * time_unit)
    // if (> 5)
    // {
    //   break;
    // }
    if (dynamixel_controller.jaw_velocity[grasp_cycle_index] < 0)
    {
      dynamixel_controller.jaw_velocity[grasp_cycle_index] = 0;
    }
    // if ( dynamixel_controller.jaw_velocity[grasp_cycle_index - 1] > dynamixel_controller.jaw_velocity[grasp_cycle_index] && grasp_cycle_index > 20)
    // {
    //   break;
    // }
    if ( dynamixel_controller.dxl_goal_current > 20 )
    {
      break;
    }

    ROS_INFO("goal_current: %f",dynamixel_controller.dxl_goal_current);
    grasp_cycle_index ++;
    
  }
  
  dynamixel_controller.gripper_write(dynamixel_controller.dxl_goal_current);

  dynamixel_controller.gripper_torqueoff();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getReadPeriod()), &DynamixelController::readCallback, &dynamixel_controller);
  ros::Timer write_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getWritePeriod()), &DynamixelController::writeCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(dynamixel_controller.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixel_controller);

  ros::spin();

  return 0;
}
