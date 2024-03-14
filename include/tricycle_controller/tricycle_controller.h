#include <queue>

#include <tricycle_controller/odometry.h>
#include <tricycle_controller/steering_limiter.h>
#include <tricycle_controller/traction_limiter.h>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>


namespace tricycle_controller
{
class TricycleController : public controller_interface::MultiInterfaceController<hardware_interface::VelocityJointInterface,
                                                                                 hardware_interface::PositionJointInterface>
{
public:
    TricycleController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(//hardware_interface::VelocityJointInterface *hw,
              hardware_interface::RobotHW* robot_hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

protected:
    /// Hardware handles:
    hardware_interface::JointHandle traction_joint_;
    hardware_interface::JointHandle steering_joint_;

    std::string name_;

    // Traction params
    double wheel_separation_;
    double wheel_radius_;

    // Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;
    bool open_loop_;
    bool real_hw_;
    bool enable_odom_tf_;
    bool odom_only_twist_;  // for doing the pose integration in separate node

    // Frame to use for the robot base:
    std::string base_frame_id_;

    // Frame to use for odometry and odom tf:
    std::string odom_frame_id_;

    // Publish ackerman velocity
    bool publish_ackermann_command_;
    std::shared_ptr<realtime_tools::RealtimePublisher<ackermann_msgs::AckermannDrive>> ackermann_command_publisher_;

    // Odometry related:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    // Function covert twist to ackermann cmd
    std::tuple<double, double> twist_to_ackermann(double linear_command, double angular_command);
    double convert_trans_rot_vel_to_steering_angle(double v, double omega, double wheel_separation);

    // Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    // Velocity command related:
    struct Commands {
        double speed;
        double angle;
        ros::Time stamp;

        Commands()
            : speed(0.0)
            , angle(0.0)
            , stamp(0.0)
        {
        }
    };

    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;
    ros::Subscriber sub_command_;

    // Create reset odometry service
    ros::ServiceServer reset_odom_service_;

    // Speed limiters
    Commands last1_cmd_;
    Commands last0_cmd_;
    TractionLimiter limiter_traction_;
    SteeringLimiter limiter_steering_;

    // Reset Odometry
    bool resetOdometryCallBack(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (AckermannDrive)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);

    /**
     * \brief Sets odometry parameters from the URDF, i.e. the traction wheel radius
     * \param root_nh Root node handle
     * \param traction_name Name of the traction joint
     * \param steering_name Name of the steering joint
     */
    bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                               const std::string& traction_name,
                               const std::string& steering_name,
                               bool lookup_wheel_radius,
                               bool lookup_wheel_separation);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
};

}  // namespace tricycle_controller
