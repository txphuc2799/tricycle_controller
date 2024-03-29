#ifndef TRICYCLE_CONTROLLER__STEERING_LIMITER_H_
#define TRICYCLE_CONTROLLER__STEERING_LIMITER_H_

#include <cmath>

namespace tricycle_controller
{

class SteeringLimiter
{
public:
    /**
     * \brief Constructor
     * \param [in] min_position Minimum position [m] or [rad]
     * \param [in] max_position Maximum position [m] or [rad]
     * \param [in] min_velocity Minimum velocity [m/s] or [rad/s]
     * \param [in] max_velocity Maximum velocity [m/s] or [rad/s]
     * \param [in] min_acceleration Minimum acceleration [m/s^2] or [rad/s^2]
     * \param [in] max_acceleration Maximum acceleration [m/s^2] or [rad/s^2]
     */
    SteeringLimiter(double min_position = NAN,
                    double max_position = NAN,
                    double min_velocity = NAN,
                    double max_velocity = NAN,
                    double min_acceleration = NAN,
                    double max_acceleration = NAN);

    /**
     * \brief Limit the position, velocity and acceleration
     * \param [in, out] p  position [m] or [rad]
     * \param [in]      p0 Previous position to p  [m] or [rad]
     * \param [in]      p1 Previous position to p0 [m] or [rad]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit(double & p, double p0, double p1, double dt);

    /**
     * \brief Limit the jerk
     * \param [in, out] p  position [m] or [rad]
     * \param [in]      p0 Previous position to p  [m] or [rad]
     * \param [in]      p1 Previous position to p0 [m] or [rad]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_position(double & p);

    /**
     * \brief Limit the velocity
     * \param [in, out] p position [m]
     * \return Limiting factor (1.0 if none)
     */
    double limit_velocity(double & p, double p0, double dt);

    /**
     * \brief Limit the acceleration
     * \param [in, out] p  Position [m] or [rad]
     * \param [in]      p0 Previous position [m] or [rad]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_acceleration(double & p, double p0, double p1, double dt);

public:
    // Position limits:
    double min_position_;
    double max_position_;

    // Velocity limits:
    double min_velocity_;
    double max_velocity_;

    // Acceleration limits:
    double min_acceleration_;
    double max_acceleration_;
};

} // namespace tricycle_controller

#endif // TRICYCLE_CONTROLLER__STEERING_LIMITER_H_
