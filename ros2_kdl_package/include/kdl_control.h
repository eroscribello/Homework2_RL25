#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"
#include "kdl/chainfksolverpos_recursive.hpp" // se serve per KDL

class KDLController
{
public:

    KDLController();
    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    KDL::JntArray velocity_ctrl_null(Eigen::Matrix<double,6,1> error_position,
                                     int Kp);



    KDL::JntArray vision_ctrl(int Kp, Eigen::Vector3d cPo, Eigen::Vector3d sd);

private:

    KDLRobot* robot_;

};

#endif

