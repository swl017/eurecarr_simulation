/**
 * @file   dynamics.h
 * @author Seungwook Lee
 * @date   2021-12-10
 * @brief  Implementing vehicle dynamics class
 */

#ifndef ES_DYNAMICS_H_
#define ES_DYNAMICS_H_

#include <iostream>
#include <type_defs.h>
#include <model_param.h>

namespace eurecarr {

class Dynamics
{
public:

    Dynamics(const double& dt, const int& model_type, const int& RK4_dt_mult);
    Dynamics(const double& dt, const int& model_type, const int& RK4_dt_mult, const States& initial_states, const Inputs& initial_inputs);
    ~Dynamics();
    void init(const double& dt, const int& model_type, const int& RK4_dt_mult, const States& initial_states, const Inputs& initial_inputs);
    void rollout(const States& states, const Inputs& inputs, const double& sim_time, States& next_states);
    void getStatesDerivatives(const States& states, const Inputs& inputs, States& states_der);
    void getNextState(const States& states, const Inputs& inputs, States& next_states);
    void EulerMethod(const States& states, const Inputs& inputs, States& next_states);
    void RK4(const States& states, const Inputs& inputs, States& next_states);
    double getdt();

private:

    double dt_;
    int model_type_;
    int RK4_dt_mult_;
    States last_states_;
    Inputs last_inputs_;

    void local2global(double x, double y, double yaw, double& x_g, double& y_g); 
    void global2local(double x, double y, double yaw, double& x_l, double& y_l);
    void simpleBicycleModel(const States &states, const Inputs &inputs, States &states_der);
    void dynamicBicycleModel(const States &states, const Inputs &inputs, States &states_der);
    void motorModel(const States &states, const Inputs &inputs, double &F_rx);
    void pacejkaTireModel(const double &alpha_f, const double &alpha_r, double &F_fy, double &F_ry);

};

}

#endif // ES_DYNAMICS_H_