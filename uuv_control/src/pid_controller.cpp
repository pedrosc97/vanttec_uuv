#include "pid_controller.hpp"

PIDController::PIDController(float _sample_time_s, const float _k_pid[3])
{
    this->sample_time_s     = _sample_time_s;
    this->k_p               = _k_pid[0];
    this->k_i               = _k_pid[1];
    this->k_d               = _k_pid[2];

    this->error             = 0;
    this->prev_error        = 0;
    this->set_point         = 0;
    this->manipulation      = 0;
    
    this->f_x               = 0;
    this->g_x               = 0;
}

PIDController::~PIDController(){}

void PIDController::CalculateManipulation(float _current_value)
{
    this->prev_error    = this->error;
    this->error         = this->set_point - _current_value;

    float error_d       = (this->error - this->prev_error) / this->sample_time_s;
    float error_i       = ((this->error + this->prev_error) / 2 * this->sample_time_s) + this->error;

    this->manipulation  = (1 / this->g_x) * (-this->f_x + this->k_p * this->error + this->k_i * error_i + this->k_d * error_d);
}