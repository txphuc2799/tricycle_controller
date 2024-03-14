#include <tricycle_controller/odometry.h>
#include <boost/bind.hpp>


namespace tricycle_controller
{
namespace bacc = boost::accumulators;

Odometry::Odometry(size_t velocity_rolling_window_size)
:   timestamp_(0.0),
    x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_(0.0),
    angular_(0.0),
    wheel_separation_(0.0),
    wheel_radius_(0.0),
    traction_old_pos_(0.0),
    velocity_rolling_window_size_(velocity_rolling_window_size),
    linear_acc_(RollingWindow::window_size = velocity_rolling_window_size),
    angular_acc_(RollingWindow::window_size = velocity_rolling_window_size),
    integrate_fun_(boost::bind(&Odometry::integrateExact, this, _1, _2, _3))
{
}

void Odometry::init(const ros::Time& time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update(double traction_pos, double steer_pos, const ros::Time& time)
{
    // Get current wheel joint positions:
    const double traction_cur_pos = traction_pos * wheel_radius_;

    // Estimate velocity of wheels using old and current position:
    const double traction_est_vel = traction_cur_pos - traction_old_pos_;

    // Update old position with current:
    traction_old_pos_ = traction_cur_pos;

    /// Compute linear and angular diff:
    const double cos_phi = std::cos(steer_pos);
    const double sin_phi = std::sin(steer_pos);

    const double linear = traction_est_vel * cos_phi;
    const double angular = traction_est_vel * sin_phi / wheel_separation_;

    double radius = 1e6;

    if (fabs(angular) >= 1e-6) {
        const double cot_phi = cos_phi / sin_phi;
        radius = cot_phi * wheel_separation_;
    }

    // Integrate odometry:
    integrate_fun_(linear, angular, radius);

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - timestamp_).toSec();
    if (dt < 0.0001)
        return false; // Interval too small to integrate with

    timestamp_ = time;

    /// Estimate speeds using a rolling mean to filter them out:
    linear_acc_(linear / dt);
    angular_acc_(angular / dt);

    linear_ = bacc::rolling_mean(linear_acc_);
    angular_ = bacc::rolling_mean(angular_acc_);

    return true;
}

void Odometry::updateOpenLoop(double linear, double angular, const ros::Time& time)
{
    /// Save last linear and angular velocity:
    linear_ = linear;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = (time - timestamp_).toSec();
    timestamp_ = time;
    integrate_fun_(linear * dt, angular * dt, linear_ / angular_);
}

void Odometry::resetOdometry()
{
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
    resetAccumulators();
}

void Odometry::setWheelParams(double wheel_separation, double wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear, double angular)
{
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_ += linear * std::cos(direction);
    y_ += linear * std::sin(direction);
    heading_ += angular;
}

void Odometry::integrateExact(double linear, double angular, double radius)
{
    if (fabs(angular) < 1e-6)
    {
        integrateRungeKutta2(linear, angular);
    }
    else
    {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = heading_;
        heading_ += angular;
        x_ += radius * (std::sin(heading_) - std::sin(heading_old));
        y_ += -radius * (std::cos(heading_) - std::cos(heading_old));
    }
}

void Odometry::resetAccumulators()
{
    linear_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    angular_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

} // namespace tricycle_controller
