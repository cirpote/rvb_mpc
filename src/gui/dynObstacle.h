#pragma once

#include <iostream>
#include <eigen3/Eigen/Geometry>
#include "../base_ibvs_controller/utils.hpp"

class DynObstacle{
public:
    DynObstacle(Eigen::Vector3d p, Eigen::Vector3d v);
    ~DynObstacle();

    Eigen::Vector3d getPose(){ return p_; };
    Eigen::Vector3d getVel(){ return v_; };
    void setPoseandVelocity(Eigen::Vector3d p, Eigen::Vector3d v);
    void Integrate(float dt);

private:

    Eigen::Vector3d p_;
    Eigen::Vector3d v_;

};
