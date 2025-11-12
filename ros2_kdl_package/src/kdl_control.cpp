

#include "kdl_control.h"
#include <stdio.h>
#include <iostream>
#include <sstream> // Necessario per std::stringstream
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"

KDLController::KDLController(){}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

/* Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

} */

KDL::JntArray KDLController::velocity_ctrl_null(Eigen::Matrix<double,6,1> error_position,
                                    int Kp)
                                                
{
    unsigned int nj = robot_->getNrJnts();




    Eigen::MatrixXd J;
    J = robot_->getEEJacobian().data;

    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(nj,nj);

    Eigen::MatrixXd JntLimits_ (nj,2);
    JntLimits_ = robot_->getJntLimits();

    Eigen::VectorXd q_min(nj);
    Eigen::VectorXd q_max(nj);
    q_min = JntLimits_.col(0);
    q_max = JntLimits_.col(1);

    Eigen::VectorXd q(nj);
    q  = robot_->getJntValues();

    double lambda = 50;

    Eigen::VectorXd q0_dot(nj);
    for (unsigned int i = 0; i<nj; i++) {
        
        double L =(q_max(i) - q_min(i))*(q_max(i) - q_min(i));

        double G = (2*q(i) - q_max(i) - q_min(i));

        double D = (q_max(i)- q(i))*(q(i)- q_min(i));

        q0_dot(i) = 1/lambda*L*G/(D*D);

    }

    Eigen::MatrixXd J_pinv = pseudoinverse(robot_->getEEJacobian().data);
    
    Eigen::VectorXd qd_vec(nj);
    qd_vec = J_pinv * error_position * Kp + (I-J_pinv*J)*q0_dot; 

    KDL::JntArray qd(nj);

    qd.data = qd_vec;

    return qd;
}





KDL::JntArray KDLController::vision_ctrl(int Kp, Eigen::Vector3d cPo,Eigen::Vector3d sd )
{
    unsigned int nj = robot_->getNrJnts();

    Eigen::Matrix<double,3,3> Rc;
    Rc = toEigen(robot_->getEEFrame().M);//assumiamo che la matrice di rotazione siano approssimabili
    Eigen::MatrixXd K(nj,nj);
    K = 3*Kp*K.Identity(nj,nj);

    Eigen::Matrix<double,6,6> R =Eigen::Matrix<double,6,6>::Zero();

    R.block<3, 3>(0, 0) = Rc;//.transpose();
    R.block<3, 3>(3, 3) = Rc;//.transpose();

    Eigen::Vector3d s;
    for (int i=0; i<3; i++){
        s(i) = cPo(i)/cPo.norm();
    }
    
    
    RCLCPP_INFO(rclcpp::get_logger("KDLController"),
                "vector s: %f %f %f",
                s(0),s(1),s(2));
    
    Eigen::Matrix<double,3,3> L1;
    L1 = -1/cPo.norm()* (Eigen::Matrix3d::Identity() - s*s.transpose());

    Eigen::Matrix3d S_skew = Eigen::Matrix3d::Zero();
    S_skew <<     0, -s.z(),  s.y(),
                 s.z(),      0, -s.x(),
                -s.y(),  s.x(),      0;


    Eigen::Matrix<double,3,3> L2;
    L2 = S_skew;

    Eigen::Matrix<double,3,6> L;

    L.block<3, 3>(0, 0) = L1;
    L.block<3, 3>(0, 3) = L2;
    
    L = L*R;

    Eigen::MatrixXd J;
    J = robot_->getEEJacobian().data;
    Eigen::MatrixXd Jc; 
    Jc  = J; //assumiamo che i due jacobiani siano uguali

    
    Eigen::MatrixXd I;
    I = Eigen::MatrixXd::Identity(nj,nj);

    Eigen::MatrixXd JntLimits_ (nj,2);
    JntLimits_ = robot_->getJntLimits();

    Eigen::VectorXd q_min(nj);
    Eigen::VectorXd q_max(nj);
    q_min = JntLimits_.col(0);
    q_max = JntLimits_.col(1);

    Eigen::VectorXd q(nj);
    q  = robot_->getJntValues();

    double lambda = 50;

    Eigen::VectorXd q0_dot(nj);
    for (unsigned int i = 0; i<nj; i++) {
        
        double L =(q_max(i) - q_min(i))*(q_max(i) - q_min(i));

        double G = (2*q(i) - q_max(i) - q_min(i));

        double D = (q_max(i)- q(i))*(q(i)- q_min(i));

        q0_dot(i) = 1/lambda*L*G/(D*D);

    }

    Eigen::MatrixXd N (nj,nj);

    N = I - pseudoinverse(J)*J;
    
    Eigen::MatrixXd J_pinv = pseudoinverse(L*J);
    KDL::JntArray qd(nj);
    qd.data =  K*J_pinv*sd + N * q0_dot;

   
    return qd;
}
