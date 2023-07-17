#ifndef PID_HPP
#define PID_HPP

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

   public:
    /*
     * @brief PID class constructor
     *
     * @param pid_gains PID gains structure
     * @param output_limits Output limits structure
     */
    PID(pid_gains_t pid_gains, output_limits_t output_limits = {0, 255});
    /*
     * @brief PID class constructor
     *
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param min_output Minimum output value for PID correction
     * @param max_output Maximum output value for PID correction
     */
    PID(float kp, float ki, float kd, float min_output = 0, float max_output = 255);

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
     */
    void set_setpoint(float setpoint);

    /*
     * @brief Getter for PID gains
     *
     * @return PID gains structure
     */
    pid_gains_t get_pid_gains(void);
    /*
     * @brief Getter for PID gains
     *
     * @return Proportional gain
     */
    float get_p_gain(void);
    /*
     * @brief Getter for PID gains
     *
     * @return Integral gain
     */
    float get_i_gain(void);
    /*
     * @brief Getter for PID gains
     *
     * @return Derivative gain
     */
    float get_d_gain(void);

    /*
     * @brief Getter for output limits
     *
     * @return Output limits structure
     */
    output_limits_t get_output_limits(void);
    /*
     * @brief Getter for output limits
     *
     * @return Minimum output value for PID correction
     */
    float get_min_output_limit(void);
    /*
     * @brief Getter for output limits
     *
     * @return Maximum output value for PID correction
     */
    float get_max_output_limit(void);

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
    bool _check_output_limits(output_limits_t &output_limits);
    bool _check_pid_gains(pid_gains_t &pid_gains);
};

#endif  // !PID_HPP
