#pragma once

#include "conversions.hpp"

using namespace std;

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
    inline double getYaw() const { return utils::yawFromQuaternion(orientation_W_B); }
    inline void getEulerAngles(Eigen::Vector3d* euler_angles) const {
        utils::getEulerAnglesFromQuaternion(orientation_W_B, euler_angles);
    }
    inline double getYawRate() const { return angular_velocity_B.z(); }
    // WARNING: sets roll and pitch to 0.
    inline void setFromYaw(double yaw) {
        orientation_W_B = utils::quaternionFromYaw(yaw);
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
    odometry->position_W = utils::vector3FromPointMsg(msg.pose.pose.position);
    odometry->orientation_W_B = utils::quaternionFromMsg(msg.pose.pose.orientation);
    odometry->velocity_B = utils::vector3FromMsg(msg.twist.twist.linear);
    odometry->angular_velocity_B = utils::vector3FromMsg(msg.twist.twist.angular);
    odometry->pose_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(msg.pose.covariance.data());
    odometry->twist_covariance_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(msg.twist.covariance.data());
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
