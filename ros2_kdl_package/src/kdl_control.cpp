#include "kdl_control.h"

KDLController::KDLController() 
{

}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}


// Inverse dynamics control in the joint space
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

// PD+ in the joint space
Eigen::VectorXd KDLController::PDCntr(KDL::JntArray &_qd, 
                                      double _Kp, double _Kd)
{

    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    //Eigen::VectorXd de = _dqd.data - dq;
    //Eigen::VectorXd ddqd = _ddqd.data;
    return _Kp*e - _Kd*dq + robot_->getGravity();
}

Eigen::VectorXd KDLController::GravityCompensation(KDL::JntArray &q)
{
    return robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

    Eigen::Matrix3d Kpp = _Kpp*Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix3d Kpo = _Kpo*Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix3d Kdp = _Kdp*Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix3d Kdo = _Kdo*Eigen::Matrix<double,3,3>::Identity();

    Eigen::Matrix<double,6,6> gainMatrixKp=Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,6> gainMatrixKd=Eigen::Matrix<double,6,6>::Zero();



    gainMatrixKp.block<3, 3>(0, 0) = Kpp; // Top-left block (position)
    gainMatrixKp.block<3, 3>(3, 3) = Kpo; // Bottom-right block (orientation)

    gainMatrixKd.block<3, 3>(0, 0) = Kdp; // Top-left block (position)
    gainMatrixKd.block<3, 3>(3, 3) = Kdo; // Bottom-right block (orientation)

    Eigen::Matrix<double,7,7> M = robot_->getJsim();


    Eigen::Matrix<double,3,1> desPos_(_desPos.p.data);
    Eigen::Matrix<double,3,1> currPos_(robot_-> getEEFrame().p.data);

    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
    Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
    Eigen::Matrix3d Rd = matrixOrthonormalization(R_d);
    Eigen::Matrix3d Re = matrixOrthonormalization(R_e);

    Eigen::Matrix<double,3,1> desVel_p(_desVel.vel.data);
    Eigen::Matrix<double,3,1> currVel_p (robot_-> getEEVelocity().vel.data);

    Eigen::Matrix<double,3,1> desVel_rot(_desVel.rot.data);
    Eigen::Matrix<double,3,1> currVel_rot(robot_->getEEVelocity().rot.data);



    Eigen::Vector3d desAcc_p(_desAcc.vel.data);
    Eigen::Vector3d desAcc_rot(_desAcc.rot.data);

    // position error 
    Eigen::Matrix<double,3,1> e_p=computeLinearError(desPos_,currPos_);
    Eigen::Matrix<double,3,1> e_o=computeOrientationError(Rd,Re);
    // velocity error 
    Eigen::Matrix<double,3,1> de_p=computeLinearError(desVel_p,currVel_p);
    Eigen::Matrix<double,3,1> de_o=computeOrientationVelocityError(desVel_rot,currVel_rot,Rd,Re);

    // final error computation 
    Eigen::Matrix<double,6,1> position_error;
    Eigen::Matrix<double,6,1> velocity_error;
    Eigen::Matrix<double,6,1> desAcc;

    position_error << e_p, e_o;
    velocity_error << de_p, de_o;
    desAcc << desAcc_p,desAcc_rot;

    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
    Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);
    Eigen::Matrix<double,6,1> Jdotqdot= toEigen(robot_->getEEJacDotqdot());

    Eigen::Matrix<double,7,1> y;

    y << Jpinv*(desAcc +gainMatrixKd*velocity_error + gainMatrixKp*position_error - Jdotqdot);

    return M * y + robot_->getGravity() + robot_->getCoriolis();

}


