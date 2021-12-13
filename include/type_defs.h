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

    struct States
    {
        // xyz position
        double x;
        double y;
        // rpy Euler angles
        double yaw;
        // xyz velocity
        double xdot;
        double ydot;
        // body frame velocity
        double ux;
        double uy;
        // rpy Euler angle rate
        double yawdot;
        
        const States operator+(const States &other) const;
        const States operator*(const double &dt) const;
        const States operator/(const double &num) const;

        void setZero() {
            x = 0.0;
            y = 0.0;
            yaw = 0.0;
            xdot = 0.0;
            ydot = 0.0;
            ux = 0.0;
            uy = 0.0;
            yawdot = 0.0;
        }
    }; // Car

    const States States::operator+( const States &other ) const
    {
        States result = *this;
        result.x += other.x;
        result.y += other.y;
        result.yaw += other.yaw;
        result.xdot += other.xdot;
        result.ydot += other.ydot;
        result.ux += other.ux;
        result.uy += other.uy;
        result.yawdot += other.yawdot;

        return result;
    }

    const States States::operator*( const double &dt ) const
    {
        States result = *this;
        result.x *= dt;
        result.y *= dt;
        result.yaw *= dt;
        result.xdot *= dt;
        result.ydot *= dt;
        result.ux *= dt;
        result.uy *= dt;
        result.yawdot *= dt;

        return result;
    }

    const States States::operator/( const double &num ) const
    {
        States result = *this;
        result.x /= num;
        result.y /= num;
        result.yaw /= num;
        result.xdot /= num;
        result.ydot /= num;
        result.ux /= num;
        result.uy /= num;
        result.yawdot /= num;

        return result;
    }

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
}

#endif // ES_TYPE_DEF_H_