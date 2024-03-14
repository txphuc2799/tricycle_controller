#ifndef TRICYCLE_CONTROLLER__TRACTION_LIMITER_H_
#define TRICYCLE_CONTROLLER__TRACTION_LIMITER_H_

#include <cmath>

namespace tricycle_controller
{
class TractionLimiter
{
public:
    /**
     * \brief Constructor
     * \param [in] min_velocity Minimum velocity [m/s] or [rad/s]
     * \param [in] max_velocity Maximum velocity [m/s] or [rad/s]
     * \param [in] min_acceleration Minimum acceleration [m/s^2] or [rad/s^2]
     * \param [in] max_acceleration Maximum acceleration [m/s^2] or [rad/s^2]
     * \param [in] min_deceleration Minimum deceleration [m/s^2] or [rad/s^2]
     * \param [in] max_deceleration Maximum deceleration [m/s^2] or [rad/s^2]
     * \param [in] min_jerk Minimum jerk [m/s^3], usually <= 0
     * \param [in] max_jerk Maximum jerk [m/s^3], usually >= 0
     */
    TractionLimiter(
      double min_velocity = NAN,
      double max_velocity = NAN,
      double min_acceleration = NAN,
      double max_acceleration = NAN,
      double min_deceleration = NAN,
      double max_deceleration = NAN,
      double min_jerk = NAN,
      double max_jerk = NAN);

    /**
     * \brief Limit the velocity and acceleration
     * \param [in, out] v  Velocity [m/s] or [rad/s]
     * \param [in]      v0 Previous velocity to v  [m/s] or [rad/s]
     * \param [in]      v1 Previous velocity to v0 [m/s] or [rad/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit(double & v, double v0, double v1, double dt);

    /**
     * \brief Limit the velocity
     * \param [in, out] v Velocity [m/s] or [rad/s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_velocity(double & v);

    /**
     * \brief Limit the acceleration
     * \param [in, out] v  Velocity [m/s] or [rad/s]
     * \param [in]      v0 Previous velocity [m/s] or [rad/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     */
    double limit_acceleration(double & v, double v0, double dt);

    /**
     * \brief Limit the jerk
     * \param [in, out] v  Velocity [m/s] or [rad/s]
     * \param [in]      v0 Previous velocity to v  [m/s] or [rad/s]
     * \param [in]      v1 Previous velocity to v0 [m/s] or [rad/s]
     * \param [in]      dt Time step [s]
     * \return Limiting factor (1.0 if none)
     * \see http://en.wikipedia.org/wiki/Jerk_%28physics%29#Motion_control
     */
    double limit_jerk(double & v, double v0, double v1, double dt);

    /**
     * @brief Clam given value to upper and lower limits.
     * 
     * @param value Input value that's possibly clamped.
     * @param lower_limit Lower limit which the value must not exceed.
     * @param upper_limit Upper limit which the value must not exceed.
     * @return double Clamped value to range in between [lower_limit, upper_limit].
     */
    double clamp(const double& value, const double& lower_limit, const double& upper_limit);

public:
    // Velocity limits:
    double min_velocity_;
    double max_velocity_;

    // Acceleration limits:
    double min_acceleration_;
    double max_acceleration_;

    // Deceleration limits:
    double min_deceleration_;
    double max_deceleration_;

    // Jerk limits:
    double min_jerk_;
    double max_jerk_;
};

}  // namespace tricycle_controller

#endif  // TRICYCLE_CONTROLLER__TRACTION_LIMITER_H_