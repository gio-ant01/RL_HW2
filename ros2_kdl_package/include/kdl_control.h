#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

/*     Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo); */

                           
    // New velocity controller with null space optimization
    Eigen::VectorXd velocity_ctrl_null(const Eigen::Vector3d &_pos_d,
                                       const Eigen::Vector3d &_pos_e,
                                       double _Kp,
                                       double _lambda);

    Eigen::VectorXd vision_ctrl(const Eigen::Vector3d &_c_Po,
                                const Eigen::Matrix3d &_R_c,
                                const Eigen::MatrixXd &_J_cam,
                                const Eigen::Vector3d &_s_d,
                                double _K,
                                double _lambda);                                                       
                                                        

private:

    KDLRobot* robot_;

    // function to compute joint limit avoidance gradient
    Eigen::VectorXd computeJointLimitGradient(double _lambda);

    //Helper function to compute skew-symmetric matrix
    Eigen::Matrix3d skew(const Eigen::Vector3d &v);

    //Helper function to compute L(s) matrix
    Eigen::MatrixXd computeInteractionMatrix(const Eigen::Vector3d &_c_Po,
                                            const Eigen::Matrix3d &_R_c);


};

#endif