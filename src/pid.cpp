#include "pid.hpp"

PID::PID(pid_gains_t pid_gains, output_limits_t output_limits, uint8_t rate) {
    _pid_gains = pid_gains;
    _output_limits = output_limits;
    _error_sum = 0;
    _prev_error = 0;
    _setpoint = 0;
    _prev_correction = 0;

    if (rate > 0) {
        _timer = new TimerAPI((1 / rate) * 1000);
        _timer->start();
    }
}

PID::PID(float kp,
         float ki,
         float kd,
         float min_output,
         float max_output,
         uint8_t rate) {
    _pid_gains = {kp, ki, kd};
    _output_limits = {min_output, max_output};
    _error_sum = 0;
    _prev_error = 0;
    _setpoint = 0;
    _prev_correction = 0;

    if (rate > 0) {
        _timer = new TimerAPI((1 / rate) * 1000);
        _timer->start();
    }
}

void PID::set_pid_gains(pid_gains_t pid_gains) { _pid_gains = pid_gains; }
void PID::set_pid_gains(float kp, float ki, float kd) { _pid_gains = {kp, ki, kd}; }

void PID::set_output_limits(output_limits_t output_limits) {
    _output_limits = output_limits;
    _check_output_limits(_output_limits);
}
void PID::set_output_limits(float min_output, float max_output) {
    _output_limits = {min_output, max_output};
    _check_output_limits(_output_limits);
}

void PID::set_setpoint(float setpoint, bool reset) {
    _setpoint = setpoint;
    if (reset) this->reset();
}

pid_gains_t PID::get_pid_gains() { return _pid_gains; }

output_limits_t PID::get_output_limits() { return _output_limits; }

float PID::get_setpoint() { return _setpoint; }

void PID::reset() {
    _error_sum = 0;
    _prev_error = 0;
    if (_timer != nullptr) _timer->reset();
}

float PID::compute(float measured_value) {
    if (_timer != nullptr && !_timer->has_elapsed()) {
        return _prev_correction;
    }
    const float kp = _pid_gains.kp;
    const float ki = _pid_gains.ki;
    const float kd = _pid_gains.kd;
    const float min_output = _output_limits.min_output;
    const float max_output = _output_limits.max_output;

    const float error = _setpoint - measured_value;

    // Eliminate possible noise on the integral term.
    if (error == 0 && _setpoint == 0) {
        reset();
    }

    _error_sum = _bound_value(_error_sum + error, min_output, max_output);

    const float p = kp * error;
    const float i = ki * _error_sum;
    const float d = kd * (error - _prev_error);

    float correction = _bound_value(p + i + d, min_output, max_output);

    _prev_correction = correction;
    _prev_error = error;

    return correction;
}

float PID::_bound_value(float value, float min_value, float max_value) {
    if (value > max_value)
        return max_value;
    else if (value < min_value)
        return min_value;
    else
        return value;
    ;
}

bool PID::_check_output_limits(output_limits_t &output_limits) {
    if (output_limits.min_output > output_limits.max_output) {
        float temp = output_limits.max_output;
        output_limits.max_output = output_limits.min_output;
        output_limits.min_output = temp;
        return true;
    }
    return false;
}
