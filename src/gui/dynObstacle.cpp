#include "dynObstacle.h"

DynObstacle::DynObstacle(Eigen::Vector3d p, Eigen::Vector3d v) : p_(p), v_(v) {

    std::cout << FGRN("Created a Dynamic Obstacle in: ") << p_.transpose() << FGRN(" with Velocity: ") << v_.transpose() << "\n"; 
}

DynObstacle::~DynObstacle(){}

void DynObstacle::setPoseandVelocity(Eigen::Vector3d p, Eigen::Vector3d v){

    p_ = p;
    v_ = v;
    std::cout << FGRN("Changed Dynamic Obstacle position in: ") << p_.transpose() << 
                 FGRN(" and velocity in: ") << v_.transpose() << "\n";
}

void DynObstacle::Integrate(float dt){

    // Check if the dynamic obstacle has a velocity
    if(v_.norm()>1e-2)
        p_ += v_*dt;
}