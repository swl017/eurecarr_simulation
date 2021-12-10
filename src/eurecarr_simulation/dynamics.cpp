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

Dynamics::Dynamics(const double& dt, const ModelType& model_type) :
dt_(dt),
model_type_(model_type)
{
    init(dt, model_type);
    std::cout << "Dynamics class initialized" << std::endl;
}

void Dynamics::init(const double& dt, const ModelType& model_type)
{
    last_states_.setZero();
    last_inputs_.setZero();
    std::cout << "Dynamics class initialized" << std::endl;
}

void Dynamics::forward(const States& states, const Inputs& inputs, States& states_der)
{
    States states_der;
    if(model_type_.type == SIMPLE_BICYCLE_MODEL)
    {
        simpleBicycleModel(states, inputs, states_der);
    }
    else
    {
        states_der.setZero();
    }

    last_states_ = states;
    last_inputs_ = inputs;

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
 * @brief Assuming the body frame origin(x,y) at the rear axis center
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

void Dynamics::motorModel(const States &states, const Inputs &inputs, double &F_rx)
{
    double ux = states.ux;
    double throttle = inputs.throttle;
    
    int sign_ux = ux >= 0 ? 1 : -1;

    // Rear wheel body x-direction force
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