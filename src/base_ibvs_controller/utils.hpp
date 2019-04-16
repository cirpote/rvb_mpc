#pragma once

using namespace std;

#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

// TXT output STUFF
#include <iostream>
#include <fstream>

/*struct UV_Bounds{

    Eigen::Vector2f u_bounds;
    Eigen::Vector2f v_bounds;

    UV_Bounds() : u_bounds(Eigen::Vector2f(0.f,0.f)), 
                  v_bounds(Eigen::Vector2f(0.f,0.f)) {}

    UV_Bounds( float* u_bnds, float* v_bnds ) {
        u_bounds = Eigen::Vector2f(u_bnds[0], u_bnds[1]);
        v_bounds = Eigen::Vector2f(v_bnds[0], v_bnds[1]);
    }
    ~UV_Bounds(){}
};*/

namespace mav_utils{

    inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
    }


    inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
    return Eigen::Vector3d(msg.x, msg.y, msg.z);
    }


    inline Eigen::Quaterniond quaternionFromMsg(
        const geometry_msgs::Quaternion& msg) {
    // Make sure this always returns a valid Quaternion, even if the message was
    // uninitialized or only approximately set.
    Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
    if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
        quaternion.setIdentity();
    } else {
        quaternion.normalize();
    }
    return quaternion;
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

namespace utils{
    inline bool exists(const std::string& name) {
        ifstream f(name.c_str());
        return f.good();
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