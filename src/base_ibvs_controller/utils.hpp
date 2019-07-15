#pragma once

using namespace std;

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

    inline bool exists(const std::string& name) {
        ifstream f(name.c_str());
        return f.good();
    }

    struct EigenOdometry {
        EigenOdometry()
            : timestamp_ns(-1),
                position_W(Eigen::Vector3d::Zero()),
                orientation_W_B(Eigen::Quaterniond::Identity()),
                velocity_B(Eigen::Vector3d::Zero()),
                angular_velocity_B(Eigen::Vector3d::Zero()) {}

        EigenOdometry(const Eigen::Vector3d& _position,
                        const Eigen::Quaterniond& _orientation,
                        const Eigen::Vector3d& _velocity_body,
                        const Eigen::Vector3d& _angular_velocity)
            : position_W(_position),
                orientation_W_B(_orientation),
                velocity_B(_velocity_body),
                angular_velocity_B(_angular_velocity) {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        int64_t
            timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
        Eigen::Vector3d position_W;
        Eigen::Quaterniond orientation_W_B;
        Eigen::Vector3d velocity_B;  // Velocity in expressed in the Body frame!
        Eigen::Vector3d angular_velocity_B;
        Eigen::Matrix<double, 6, 6> pose_covariance_;
        Eigen::Matrix<double, 6, 6> twist_covariance_;

        // Accessors for making dealing with orientation/angular velocity easier.
        inline double getYaw() const { return yawFromQuaternion(orientation_W_B); }
        inline void getEulerAngles(Eigen::Vector3d* euler_angles) const {
            getEulerAnglesFromQuaternion(orientation_W_B, euler_angles);
        }
        inline double getYawRate() const { return angular_velocity_B.z(); }
        // WARNING: sets roll and pitch to 0.
        inline void setFromYaw(double yaw) {
            orientation_W_B = quaternionFromYaw(yaw);
        }
        inline void setFromYawRate(double yaw_rate) {
            angular_velocity_B.x() = 0.0;
            angular_velocity_B.y() = 0.0;
            angular_velocity_B.z() = yaw_rate;
        }

        inline Eigen::Vector3d getVelocityWorld() const {
            return orientation_W_B * velocity_B;
        }
        inline void setVelocityWorld(const Eigen::Vector3d& velocity_world) {
            velocity_B = orientation_W_B.inverse() * velocity_world;
        }
    };

    inline void eigenOdometryFromMsg(const nav_msgs::Odometry& msg,
                                 EigenOdometry* odometry) {
        assert(odometry != NULL);
        odometry->timestamp_ns = msg.header.stamp.toNSec();
        odometry->position_W = vector3FromPointMsg(msg.pose.pose.position);
        odometry->orientation_W_B = quaternionFromMsg(msg.pose.pose.orientation);
        odometry->velocity_B = vector3FromMsg(msg.twist.twist.linear);
        odometry->angular_velocity_B = vector3FromMsg(msg.twist.twist.angular);
        odometry->pose_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(msg.pose.covariance.data());
        odometry->twist_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(msg.twist.covariance.data());
    }




}

/* FOREGROUND */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

#define BOLD(x)	"\x1B[1m" x RST
#define UNDL(x)	"\x1B[4m" x RST

enum Verbosity_Level {
    SILENT = 0,
    VERBOSE = 1,
    FEMMINA_CAGACAZZI = 2
};
