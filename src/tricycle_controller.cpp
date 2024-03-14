#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <boost/assign.hpp>
#include <stdexcept>
#include <string>
#include <ros/ros.h>
#include <tuple>

#include <tricycle_controller/tricycle_controller.h>
#include <pluginlib/class_list_macros.hpp>

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
    if (!link)
    {
        ROS_ERROR("Link pointer is null.");
        return false;
    }

    if (!link->collision)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
        return false;
    }

    if (!link->collision->geometry)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
        return false;
    }

    if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
    {
        ROS_ERROR_STREAM("Link " << link->name << " does not have cylinder geometry");
        return false;
    }

    return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
static bool getWheelRadius(const urdf::LinkConstSharedPtr& wheel_link, double& wheel_radius)
{
    if (!isCylinder(wheel_link))
    {
        ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder!");
        return false;
    }

    wheel_radius = (static_cast<urdf::Cylinder*>(wheel_link->collision->geometry.get()))->radius;
    return true;
}

namespace tricycle_controller
{
TricycleController::TricycleController():
    wheel_separation_(0.0),
    wheel_radius_(0.0),
    open_loop_(false),
    enable_odom_tf_(false),
    odom_only_twist_(true),
    cmd_vel_timeout_(0.5),
    base_frame_id_("base_link"),
    odom_frame_id_("odom"),
    publish_ackermann_command_(true)
    {}

bool TricycleController::init(hardware_interface::RobotHW* robot_hw,
                              ros::NodeHandle& root_nh,
                              ros::NodeHandle &controller_nh)
{
    typedef hardware_interface::VelocityJointInterface VelIface;
    typedef hardware_interface::PositionJointInterface PosIface;
    typedef hardware_interface::JointStateInterface StateIface;

    // Get multiple types of hardware_interface
    VelIface *vel_joint_if = robot_hw->get<VelIface>(); // vel for traction
    PosIface *pos_joint_if = robot_hw->get<PosIface>(); // pos for steering


    const std::string complete_ns = controller_nh.getNamespace();

    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    // Traction name
    std::string traction_name = "traction_joint";
    controller_nh.param("traction_joint_name", traction_name, traction_name);

    // Steering name
    std::string steering_name = "steering_joint";
    controller_nh.param("steering_joint_name", steering_name, steering_name);

    // Odometry related:
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    controller_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_INFO_STREAM_NAMED(name_, "Velocity rolling window size of " << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "base_frame_id set to " << base_frame_id_);

    controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
    ROS_INFO_STREAM_NAMED(name_, "odom_frame_id set to " << odom_frame_id_);

    controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_INFO_STREAM_NAMED(name_, "enable_odom_tf set to " << (enable_odom_tf_?"True":"False"));

    controller_nh.param("odom_only_twist", odom_only_twist_, odom_only_twist_);
    ROS_INFO_STREAM_NAMED(name_, "odom_only_twist set to " << (odom_only_twist_?"True":"False"));

    controller_nh.param("publish_ackermann_command", publish_ackermann_command_, publish_ackermann_command_);
    ROS_INFO_STREAM_NAMED(name_, "publish_ackermann_command set to " << (publish_ackermann_command_?"True":"False"));

    // Traction limits:
    controller_nh.param("traction/max_velocity"    , limiter_traction_.max_velocity_    , limiter_traction_.max_velocity_     );
    controller_nh.param("traction/min_velocity"    , limiter_traction_.min_velocity_    , -limiter_traction_.max_velocity_    );
    controller_nh.param("traction/max_acceleration", limiter_traction_.max_acceleration_, limiter_traction_.max_acceleration_ );
    controller_nh.param("traction/min_acceleration", limiter_traction_.min_acceleration_, -limiter_traction_.max_acceleration_);
    controller_nh.param("traction/max_deceleration", limiter_traction_.max_deceleration_, limiter_traction_.max_deceleration_ );
    controller_nh.param("traction/min_deceleration", limiter_traction_.min_deceleration_, -limiter_traction_.max_deceleration_);
    controller_nh.param("traction/max_jerk"        , limiter_traction_.max_jerk_        , limiter_traction_.max_jerk_         );
    controller_nh.param("traction/min_jerk"        , limiter_traction_.min_jerk_        , -limiter_traction_.max_jerk_        );

    // Steering limits:
    controller_nh.param("steering/max_position"    , limiter_steering_.max_position_    , limiter_steering_.max_position_     );
    controller_nh.param("steering/min_position"    , limiter_steering_.min_position_    , -limiter_steering_.max_position_    );
    controller_nh.param("steering/max_velocity"    , limiter_steering_.max_velocity_    , limiter_steering_.max_velocity_     );
    controller_nh.param("steering/min_velocity"    , limiter_steering_.min_velocity_    , -limiter_steering_.max_velocity_    );
    controller_nh.param("steering/max_acceleration", limiter_steering_.max_acceleration_, limiter_steering_.max_acceleration_ );
    controller_nh.param("steering/min_acceleration", limiter_steering_.min_acceleration_, -limiter_steering_.max_acceleration_);
    

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_wheel_separation = !controller_nh.getParam("wheel_separation", wheel_separation_);
    bool lookup_wheel_radius     = !controller_nh.getParam("wheel_radius", wheel_radius_);

    if (!setOdomParamsFromUrdf(root_nh,
                               traction_name,
                               steering_name,
                               lookup_wheel_separation,
                               lookup_wheel_radius))
    {
      return false;
    }

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    odometry_.setWheelParams(wheel_separation_, wheel_radius_);
    ROS_INFO_STREAM_NAMED(name_,
                          "Odometry params : wheel separation: " << wheel_separation_
                          << ", wheel radius: " << wheel_radius_);

    setOdomPubFields(root_nh, controller_nh);

    // Traction wheel
    ROS_INFO_STREAM_NAMED(name_,
                          "Adding the traction wheel with joint name: " << traction_name);
    traction_joint_ = vel_joint_if->getHandle(traction_name); // throws on failure

    // Steer
    ROS_INFO_STREAM_NAMED(name_,
                          "Adding the steer with joint name: " << steering_name);
    steering_joint_ = pos_joint_if->getHandle(steering_name); // throws on failure

    // Publish ackermann command
    if (publish_ackermann_command_)
    {
        ackermann_command_publisher_.reset(new realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDrive>
                                           (controller_nh, "cmd_ackermann", 100));
    }

    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &TricycleController::cmdVelCallback, this);

    // Create service reset odometry
    reset_odom_service_ = controller_nh.advertiseService("reset_odometry", &TricycleController::resetOdometryCallBack, this);

    ROS_INFO_STREAM_NAMED(name_, "Finished tricycle drive controller initialization");
    return true;
}

void TricycleController::update(const ros::Time& time, const ros::Duration& period)
{   
    // Get current position of joints
    double traction_pos = traction_joint_.getPosition();
    double steering_pos = steering_joint_.getPosition();

    // COMPUTE AND PUBLISH ODOMETRY
    if (open_loop_)
    {
        odometry_.updateOpenLoop(last0_cmd_.speed, last0_cmd_.angle, time);
    }
    else{
        if (std::isnan(traction_pos) || std::isnan(steering_pos))
            return;

        // Estimate linear and angular velocity using joint information
        odometry_.update(traction_pos, steering_pos, time);
    }

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
        last_state_publish_time_ += publish_period_;
        // Compute and store orientation info
        const geometry_msgs::Quaternion orientation(
                tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

        // Populate odom message and publish
        if (odom_pub_->trylock())
        {
            odom_pub_->msg_.header.stamp = time;
            if (!odom_only_twist_){
                odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
                odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
                odom_pub_->msg_.pose.pose.orientation = orientation;
            }
            odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinear();
            odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
            odom_pub_->unlockAndPublish();
        }

        // Publish tf /odom frame
        if (enable_odom_tf_ && tf_odom_pub_->trylock())
        {
            geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
            odom_frame.header.stamp = time;
            odom_frame.transform.translation.x = odometry_.getX();
            odom_frame.transform.translation.y = odometry_.getY();
            odom_frame.transform.rotation = orientation;
            tf_odom_pub_->unlockAndPublish();
        }
    }

    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_) {
        curr_cmd.speed = 0.0;
        curr_cmd.angle = 0.0;
    }

    // Compute wheel velocity and angle
    std::tie(curr_cmd.angle, curr_cmd.speed) = twist_to_ackermann(curr_cmd.speed, curr_cmd.angle);

    // Reduce wheel speed until the target angle has been reached
    double alpha_delta = abs(curr_cmd.angle - steering_pos);
    double scale;
    if (alpha_delta < M_PI / 6)
    {
        scale = 1;
    }
    else if (alpha_delta > M_PI_2)
    {
        scale = 0.01;
    }
    else
    {
        // TODO(anyone): find the best function, e.g convex power functions
        scale = std::cos(alpha_delta);
    }

    curr_cmd.speed *= scale;

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_traction_.limit(curr_cmd.speed, last0_cmd_.speed, last1_cmd_.speed, cmd_dt);
    limiter_steering_.limit(curr_cmd.angle, last0_cmd_.angle, last1_cmd_.angle, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    ackermann_msgs::AckermannDrive ackermann_command;
    // speed in AckermannDrive is defined as desired forward speed (m/s) but it is used here as wheel
    // speed (rad/s)
    ackermann_command.speed = curr_cmd.speed;
    ackermann_command.steering_angle = curr_cmd.angle;

    //  Publish ackermann command
    if (publish_ackermann_command_ && ackermann_command_publisher_->trylock())
    {
        // speed in AckermannDrive is defined desired forward speed (m/s) but we use it here as wheel
        // speed (rad/s)
        ackermann_command_publisher_->msg_.speed = curr_cmd.speed;
        ackermann_command_publisher_->msg_.steering_angle = curr_cmd.angle;
        ackermann_command_publisher_->unlockAndPublish();
    }

    // Set Command
    traction_joint_.setCommand(curr_cmd.speed);
    steering_joint_.setCommand(curr_cmd.angle);
}

void TricycleController::starting(const ros::Time& time)
{
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
}

void TricycleController::stopping(const ros::Time&)
{
    brake();
}

void TricycleController::brake()
{
    odometry_.resetOdometry();

    const double wheel_vel = 0.0;
    const double steer_pos = 0.0;

    traction_joint_.setCommand(wheel_vel);
    steering_joint_.setCommand(steer_pos);
}

void TricycleController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_covariance_diagonal_;
    controller_nh.getParam("pose_covariance_diagonal", pose_covariance_diagonal_);
    ROS_ASSERT(pose_covariance_diagonal_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_covariance_diagonal_.size() == 6);
    for (int i = 0; i < pose_covariance_diagonal_.size(); ++i)
        ROS_ASSERT(pose_covariance_diagonal_[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_covariance_diagonal_;
    controller_nh.getParam("twist_covariance_diagonal", twist_covariance_diagonal_);
    ROS_ASSERT(twist_covariance_diagonal_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_covariance_diagonal_.size() == 6);
    for (int i = 0; i < twist_covariance_diagonal_.size(); ++i)
        ROS_ASSERT(twist_covariance_diagonal_[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // clang-format off
    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_covariance_diagonal_[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_covariance_diagonal_[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_covariance_diagonal_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_covariance_diagonal_[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_covariance_diagonal_[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_covariance_diagonal_[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_covariance_diagonal_[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_covariance_diagonal_[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_covariance_diagonal_[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_covariance_diagonal_[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_covariance_diagonal_[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_covariance_diagonal_[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
    // clang-format on
}

bool TricycleController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                                               const std::string& traction_name,
                                               const std::string& steering_name,
                                               bool lookup_wheel_radius,
                                               bool lookup_wheel_separation)
{
    if (!(lookup_wheel_radius || lookup_wheel_separation))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    urdf::JointConstSharedPtr traction_joint(model->getJoint(traction_name));
    urdf::JointConstSharedPtr steering_joint(model->getJoint(steering_name));

    if (lookup_wheel_separation)
    {
        if (!traction_joint)
        {
            ROS_ERROR_STREAM_NAMED(name_, traction_name
                                << " couldn't be retrieved from model description");
            return false;
        }

        if (!steering_joint)
        {
            ROS_ERROR_STREAM_NAMED(name_, steering_name
                                << " couldn't be retrieved from model description");
            return false;
        }

        ROS_INFO_STREAM("traction wheel to origin: "
                        << traction_joint->parent_to_joint_origin_transform.position.x << ","
                        << traction_joint->parent_to_joint_origin_transform.position.y << ", "
                        << traction_joint->parent_to_joint_origin_transform.position.z);

        ROS_INFO_STREAM("steering to origin: "
                        << steering_joint->parent_to_joint_origin_transform.position.x << ","
                        << steering_joint->parent_to_joint_origin_transform.position.y << ", "
                        << steering_joint->parent_to_joint_origin_transform.position.z);
    }

    if (lookup_wheel_radius)
    {
        // Get traction wheel radius
        if (!getWheelRadius(model->getLink(traction_joint->child_link_name), wheel_radius_))
        {
            ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << traction_name << " wheel radius");
            return false;
        }
        ROS_INFO_STREAM("Retrieved wheel_radius: " << wheel_radius_);
    }

    return true;
}

void TricycleController::cmdVelCallback(const geometry_msgs::Twist& command)
{
    if (isRunning()) {
        command_struct_.angle = command.angular.z;
        command_struct_.speed = command.linear.x;
        command_struct_.stamp = ros::Time::now();
        command_.writeFromNonRT(command_struct_);
        ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
                << "Ang: " << command_struct_.angle << ", "
                << "Speed: " << command_struct_.speed << ", "
                << "Stamp: " << command_struct_.stamp);
    } else {
        ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
}

bool TricycleController::resetOdometryCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    odometry_.resetOdometry();
    ROS_INFO("Odometry successfully reset");
    return true;
}

std::tuple<double, double> TricycleController::twist_to_ackermann(double Vx, double theta_dot)
{
    // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
    double alpha, Ws;

    if (Vx == 0 && theta_dot != 0)
    {  // is spin action
        alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
        Ws = abs(theta_dot) * wheel_separation_ / wheel_radius_;
        return std::make_tuple(alpha, Ws);
    }

    alpha = convert_trans_rot_vel_to_steering_angle(Vx, theta_dot, wheel_separation_);
    Ws = Vx / (wheel_radius_ * std::cos(alpha));
    return std::make_tuple(alpha, Ws);
}

double TricycleController::convert_trans_rot_vel_to_steering_angle(double Vx, double theta_dot, double wheel_separation)
{
    if (theta_dot == 0 || Vx == 0)
    {
        return 0;
    }
    return std::atan(theta_dot * wheel_separation_ / Vx);
}

}   // namespace tricycle_controller

PLUGINLIB_EXPORT_CLASS(tricycle_controller::TricycleController, controller_interface::ControllerBase);