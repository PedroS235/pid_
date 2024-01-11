#ifndef PID_HPP
#define PID_HPP

#include "timer_api.hpp"

/*
 * @brief PID gains structure
 *
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

/*
 * @brief PID output limits structure
 *
 * @param min_output Minimum output value for PID correction
 * @param max_output Maximum output value for PID correction
 */
typedef struct {
    float min_output;
    float max_output;
} output_limits_t;

/*
 * @brief PID class
 * @details This class implements a PID controller where the compute method
 * needs to be called at every main loop iteration.
 */
class PID {
   private:
    pid_gains_t _pid_gains;
    output_limits_t _output_limits;
    float _error_sum, _prev_error, _setpoint;
    TimerAPI *_timer = nullptr;
    float _prev_correction;

   public:
    /*
     * @brief PID class constructor
     *
     * @param pid_gains PID gains structure
     * @param output_limits Output limits structure. [Default: (0-255)]
     * @param rate Rate (in Hz) at which the new correction is computed. If set to 0,
     * the correction is computed at every call of the compute method. [Default: 20Hz]
     */
    PID(pid_gains_t pid_gains,
        output_limits_t output_limits = {0, 255},
        uint8_t rate = 20);
    /*
     * @brief PID class constructor
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param min_output Minimum output value for PID correction. [Default: 0]
     * @param max_output Maximum output value for PID correction. [Default: 255]
     * @param rate Rate (in Hz) at which the new correction is computed. If set to 0,
     * the correction is computed at every call of the compute method. [Default: 20Hz]
     */
    PID(float kp,
        float ki,
        float kd,
        float min_output = 0,
        float max_output = 255,
        uint8_t rate = 20);

    /*
     * @brief Setter for output limits
     *
     * @param output_limits Output limits structure
     */
    void set_output_limits(output_limits_t output_limits);
    /*
     * @brief Setter for output limits
     *
     * @param min_output Minimum output value for PID correction
     * @param max_output Maximum output value for PID correction
     */
    void set_output_limits(float min_output, float max_output);

    /*
     * @brief Setter for PID gains
     *
     * @param pid_gains PID gains structure
     */
    void set_pid_gains(pid_gains_t pid_gains);
    /*
     * @brief Setter for PID gains
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void set_pid_gains(float kp, float ki, float kd);

    /*
     * @brief Setter for setpoint
     *
     * @param setpoint Setpoint value
     * @param reset If true, the PID controller is reset. [Default: false]
     */
    void set_setpoint(float setpoint, bool reset = false);

    /*
     * @brief Getter for PID gains
     *
     * @return PID gains structure
     */
    pid_gains_t get_pid_gains(void);

    /*
     * @brief Getter for output limits
     *
     * @return Output limits structure
     */
    output_limits_t get_output_limits(void);

    /*
     * @brief Getter for setpoint
     *
     * @return Setpoint value
     */
    float get_setpoint(void);

    /*
     * @brief Resets the PID controller
     */
    void reset(void);

    /*
     * @brief Computes the PID correction based on the measured value.
     *
     * @param measured_value Measured value
     * @return PID correction value which is bounded by the output limits
     */
    float compute(float measured_value);

   private:
    /*
     * @brief Bounds a value between a minimum and a maximum value
     *
     * @param value Value to be bounded
     * @param min_value Minimum value
     * @param max_value Maximum value
     * @return Bounded value
     */
    float _bound_value(float value, float min_value, float max_value);

    /*
     * @brief Checks if the output limits are valid
     *
     * @param output_limits Output limits structure
     * @return True if the output limits are valid, false otherwise
     */
    bool _check_output_limits(output_limits_t &output_limits);
};

#endif  // !PID_HPP
