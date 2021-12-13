/**
 * @file   dynamics.cpp
 * @author Seungwook Lee
 * @date   2021-12-10
 * @brief  Implementing vehicle dynamics class
 */

#include <stdio.h>
#include <cmath>
#include <eurecarr_simulation/dynamics.h>

namespace eurecarr {

/**
 * @brief zero state zero input initialize
 * @param dt The time from current state to next state
 * @param model_type SIMPLE_BICYCLE_MODEL = 1, DYNAMIC_BICYCLE_MODEL = 2
 * @param RK4_dt_mult <= 0: Euler method(fast), >= 1: fine_dt = dt / RK4_dt_mult (Runge-Kutta 4th Order Integration Method)
 */
Dynamics::Dynamics(const double& dt, const int& model_type, const int& RK4_dt_mult)
{
    States initial_states;
    Inputs initial_inputs;
    initial_states.setZero();
    initial_inputs.setZero();
    init(dt, model_type, RK4_dt_mult, initial_states, initial_inputs);
    std::cout << "Dynamics class initialized" << std::endl;
}

/**
 * @brief non-zero state non-zero input initialize
 * @param dt The time from current state to next state
 * @param model_type SIMPLE_BICYCLE_MODEL = 1, DYNAMIC_BICYCLE_MODEL = 2
 * @param RK4_dt_mult <= 0: Euler method(fast), >= 1: fine_dt = dt / RK4_dt_mult (Runge-Kutta 4th Order Integration Method)
 * @param initial_states x, y, yaw, xdot, ydot, ux, uy, yawdot
 * @param initial_inputs steering, throttle
 */
Dynamics::Dynamics(const double& dt, 
                    const int& model_type, 
                    const int& RK4_dt_mult,
                    const States& initial_states,
                    const Inputs& initial_inputs)
{
    init(dt, model_type, RK4_dt_mult, initial_states, initial_inputs);
    std::cout << "Dynamics class initialized" << std::endl;
}

Dynamics::~Dynamics()
{}

/**
 * @brief Can manually initiailize variables with this function
 */
void Dynamics::init(const double& dt, const int& model_type, const int& RK4_dt_mult, const States& initial_states, const Inputs& initial_inputs)
{
    dt_ = dt;
    model_type_ = model_type;
    RK4_dt_mult_ = RK4_dt_mult;
    last_states_ = initial_states;
    last_inputs_ = initial_inputs;
    std::cout << "Dynamics class initialized" << std::endl;
}

/**
 * @brief rollout for sim_time(sec) with constant inputs
 */
void Dynamics::rollout(const States& states, const Inputs& inputs, const double& sim_time, States& next_states)
{
    States _states = states;
    last_states_ = states;
    int num_steps = (int) sim_time / dt_;
    for(int i = 0; i < num_steps; i++)
    {
        getNextState(_states, inputs, next_states);
        _states = next_states;
    }
}

/**
 * @brief Compute state derivatives using current states and inputs (Continous)
 */
void Dynamics::getStatesDerivatives(const States& states, const Inputs& inputs, States& states_der)
{
    if(model_type_ == SIMPLE_BICYCLE_MODEL)
    {
        simpleBicycleModel(states, inputs, states_der);
    }
    else if(model_type_ == DYNAMIC_BICYCLE_MODEL)
    {
        dynamicBicycleModel(states, inputs, states_der);
    }
    else
    {
        states_der.setZero();
    }

    last_states_ = states;
    last_inputs_ = inputs;
}

/**
 * @brief Compute next states using current states and inputs (Discrete)
 */
void Dynamics::getNextState(const States& states, const Inputs& inputs, States& next_states)
{

    if(RK4_dt_mult_ <= 0)
    {
        EulerMethod(states, inputs, next_states);
    }
    else
    {
        RK4(states, inputs, next_states);
    }
}

void Dynamics::EulerMethod(const States& states, const Inputs& inputs, States& next_states)
{
    States states_der;
    getStatesDerivatives(states, inputs, states_der);
    next_states = states + states_der * dt_;
}

// Runge-Kutta 4th Order Integration Method
void Dynamics::RK4(const States& states, const Inputs& inputs, States& next_states)
{
    if(RK4_dt_mult_ > 0)
    {
        double fine_dt = dt_ / RK4_dt_mult_;
        States states_der, _states = states;
        States k1, k2, k3, k4;
        for(int i = 0; i < RK4_dt_mult_; i++)
        {
            getStatesDerivatives(_states, inputs, k1);
            getStatesDerivatives(_states + k1 * (fine_dt / 2.0), inputs, k2);
            getStatesDerivatives(_states + k2 * (fine_dt / 2.0), inputs, k3);
            getStatesDerivatives(_states + k3 * (fine_dt), inputs, k4);
            _states = _states + (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0) * fine_dt;
        }
        next_states = _states;
    }
    else
    {
        std::cout << "Set proper RK4_dt_mult and try again." << std::endl;
    }
}

double Dynamics::getdt()
{
    return dt_;
}

/**
 * @brief Assuming the body frame origin(x,y) at the rear axis center
 */
void Dynamics::simpleBicycleModel(const States &states, const Inputs &inputs, States &states_der)
{
    double steering_angle = inputs.steering;
    double last_steering_angle = last_inputs_.steering;

    double ux = states.ux;
    double uy = states.uy;
    double yaw = states.yaw;
    
    double x_dot, y_dot, yaw_dot, ux_dot, uy_dot, yaw_dotdot;

    local2global(ux, uy, yaw, x_dot, y_dot);
    yaw_dot = ux * sin(steering_angle) / AXIS_DISTANCE;

    double F_rx;
    motorModel(states, inputs, F_rx);
    ux_dot  = F_rx / m;
    uy_dot  = 0.0;
    yaw_dotdot = 1.0 / AXIS_DISTANCE
                 * ((ux * steering_angle - last_steering_angle) / dt_
                     * 1.0 / cos(steering_angle*steering_angle)
                     + ux_dot * tan(steering_angle));

    states_der.x      = x_dot;
    states_der.y      = y_dot;
    states_der.yaw    = yaw_dot;
    states_der.ux     = ux_dot;
    states_der.uy     = uy_dot;
    states_der.yawdot = yaw_dotdot;
}

/**
 * @brief Assuming the body frame origin(x,y) at the center of the mass
 */
void Dynamics::dynamicBicycleModel(const States &states, const Inputs &inputs, States &states_der)
{
    double steering = inputs.steering;
    double throttle = inputs.throttle;
    double x        = states.x;
    double y        = states.y;
    double yaw      = states.yaw;
    double ux       = states.ux;
    double uy       = states.uy;
    double yaw_dot  = states.yawdot;

    double alpha_f = atan2(yaw_dot * COG_TO_FRONT_AXIS + uy, ux) + steering;
    double alpha_r = atan2(yaw_dot * COG_TO_REAR_AXIS - uy, ux);

    alpha_f = std::min(std::max(alpha_f, -0.6), 0.6);
    alpha_r = std::min(std::max(alpha_r, -0.6), 0.6);

    double F_fy, F_ry, F_rx;
    pacejkaTireModel(alpha_f, alpha_r, F_fy, F_ry);
    motorModel(states, inputs, F_rx);

    double x_dot, y_dot;
    double ux_dot, uy_dot, yaw_dotdot;
    local2global(ux, uy, yaw, x_dot, y_dot);
    ux_dot = 1.0 / m * (F_rx - F_fy * sin(steering) + m * uy * yaw_dot);
    uy_dot = 1.0 / m * (F_ry + F_fy * cos(steering) - m * ux * yaw_dot);
    yaw_dotdot = 1.0 / Iz * (F_fy * COG_TO_FRONT_AXIS * cos(steering) - F_ry * COG_TO_REAR_AXIS);

    states_der.x      = x_dot;
    states_der.y      = y_dot;
    states_der.yaw    = yaw_dot;
    states_der.ux     = ux_dot;
    states_der.uy     = uy_dot;
    states_der.yawdot = yaw_dotdot;

}

void Dynamics::pacejkaTireModel(const double &alpha_f, const double &alpha_r, double &F_fy, double &F_ry)
{
    F_fy = Df * sin(Cf * atan(Bf * alpha_f));
    F_ry = Dr * sin(Cr * atan(Br * alpha_r));
}

void Dynamics::motorModel(const States &states, const Inputs &inputs, double &F_rx)
{
    double ux = states.ux;
    double throttle = inputs.throttle;
    
    int sign_ux = ux >= 0 ? 1 : -1;

    // Rear wheel body x-direction force.
    // Using sign of body frame velocity ux for forward and backward movement.
    F_rx = (Cm1 - Cm2 * abs(ux)) * throttle - (Cr0 + Cr2 * ux * ux) * sign_ux;
}


void Dynamics::local2global(double x, double y, double yaw, double& x_g, double& y_g)
{
    x_g = cos(yaw) * x - sin(yaw) * y;
    y_g = sin(yaw) * x + cos(yaw) * y;
}

void Dynamics::global2local(double x, double y, double yaw, double& x_l, double& y_l)
{
    x_l = cos(-yaw) * x - sin(-yaw) * y;
    y_l = sin(-yaw) * x + cos(-yaw) * y; 
}

}

int main()
{
    /** Pseudo code **/

    // dynamics = Dynamics(dynamics_dt, SIMPLE_BICYCLE_MODEL, 10);
    // States next_states, states = getState(); // real current states
    // Inputs inputs;
    // while(!rollout_done)
    // {
    //     sim_time = frenet_dt;
    //     inputs = getControlInputs(states, path);
    //     dynamics.rollout(states, inputs, sim_time, next_states);
    //     states = next_states;
    //     rollout_done = done(state, path) ? true : false;
    // }
    // final_states = next_states
}