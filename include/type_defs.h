/**
 * @file   type_defs.h
 * @author Seungwook Lee
 * @date   2021-12-10
 * @brief  Type definitions for vehicle dynamics class
 */

#ifndef ES_TYPE_DEF_H_
#define ES_TYPE_DEF_H_


namespace eurecarr {
    const int STRAIGHT_LINEAR_MODEL = 0;
    const int SIMPLE_BICYCLE_MODEL  = 1;
    const int DYNAMIC_BICYCLE_MODEL = 2;

    struct FullState
    {
        // xyz position
        double x;
        double y;
        double z;
        // rpy Euler angles
        double roll;
        double pitch;
        double yaw;
        // quaternions
        double qx;
        double qy;
        double qz;
        double qw;
        // xyz velocity
        double xdot;
        double ydot;
        double zdot;
        // body frame velocity
        double ux;
        double uy;
        double uz;
        // rpy Euler angle rate
        double rolldot;
        double pitchdot;
        double yawdot;
        // current command
        double steering;
        double throttle;

        void setZero() {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            roll = 0.0;
            pitch = 0.0;
            yaw = 0.0;
            qx = 0.0;
            qy = 0.0;
            qz = 0.0;
            qw = 1.0;
            xdot = 0.0;
            ydot = 0.0;
            zdot = 0.0;
            ux = 0.0;
            uy = 0.0;
            uz = 0.0;
            rolldot = 0.0;
            pitchdot = 0.0;
            yawdot = 0.0;
            steering = 0.0;
            throttle = 0.0;
        }
    };

    struct States
    {
        // xyz position
        double x;
        double y;
        // rpy Euler angles
        double yaw;
        // quaternions
        double qx;
        double qy;
        double qz;
        double qw;
        // xyz velocity
        double xdot;
        double ydot;
        // body frame velocity
        double ux;
        double uy;
        // rpy Euler angle rate
        double yawdot;

        void setZero() {
            x = 0.0;
            y = 0.0;
            yaw = 0.0;
            qx = 0.0;
            qy = 0.0;
            qz = 0.0;
            qw = 1.0;
            xdot = 0.0;
            ydot = 0.0;
            ux = 0.0;
            uy = 0.0;
            yawdot = 0.0;
        }
    }; // Car

    struct Inputs
    {
        // current command
        double steering; // front wheel angle
        double throttle;

        void setZero() {
            steering = 0.0;
            throttle = 0.0;
        }
    }; // Car

    struct ModelType
    {
        // @todo Move model type constants here
        int type;
    };
}

#endif // ES_TYPE_DEF_H_