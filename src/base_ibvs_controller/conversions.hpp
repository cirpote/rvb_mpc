#pragma once

#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <tf/LinearMath/Quaternion.h>
#include <iostream>
#include <fstream>

// TXT output STUFF
#include <iostream>
#include <fstream>

namespace utils{

    inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
    }


    inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
    }


    inline Eigen::Quaterniond quaternionFromMsg(const geometry_msgs::Quaternion& msg) {
        Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
        if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
            quaternion.setIdentity();
        } else {
            quaternion.normalize();
        }
        return quaternion;
    }

    inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    inline Eigen::Quaterniond quaternionFromMsg(const tf::Quaternion& msg) {
        Eigen::Quaterniond quaternion(msg.getW(), msg.getX(), msg.getY(), msg.getZ());
        if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
            quaternion.setIdentity();
        } else {
            quaternion.normalize();
        }
        return quaternion;
    }

    inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q, Eigen::Vector3d* euler_angles) {

        assert(euler_angles != NULL);

        *euler_angles << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
            asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
            atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    inline double rollFromQuaternion(const Eigen::Quaterniond& q) {
        return atan2(2.0*(q.x()*q.w() + q.y()*q.z()),
                    1 - 2*(q.x()*q.x() + q.y()*q.y()));
    }  

    inline double pitchFromQuaternion(const Eigen::Quaterniond& q) {
        return asin(2.0*(q.w()*q.y() - q.x()*q.z()));
    }


    inline double yawFromTfQuaternion(const tf::Quaternion& q) {
    return atan2(2.0 * (q.getW() * q.getZ() + q.getX() * q.getY()),
                 1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ()));
    }

    inline double rollFromTfQuaternion(const tf::Quaternion& q) {
        return atan2(2.0*(q.getX()*q.getW() + q.getY()*q.getZ()),
                    1 - 2*(q.getX()*q.getX() + q.getY()*q.getY()));
    }  

    inline double pitchFromTfQuaternion(const tf::Quaternion& q) {
        return asin(2.0*(q.getW()*q.getY() - q.getX()*q.getZ()));
    }

    inline Eigen::Matrix3d fromEulerAngToRotMat(const Eigen::Vector3d& att){

        Eigen::AngleAxisd rollAngle(att(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(att(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(att(2), Eigen::Vector3d::UnitZ());
        return Eigen::Matrix3d( yawAngle * pitchAngle * rollAngle);
    }

    inline geometry_msgs::Point fromEigenVectorToPoint( const Eigen::Vector3d& p ){
        geometry_msgs::Point point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        return point;
    }

    inline geometry_msgs::Quaternion fromEigenQuaternionrToQuaternion( const Eigen::Quaterniond& q ){
        geometry_msgs::Quaternion quat;
        quat.w = q.w();
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        return quat;
    }
}