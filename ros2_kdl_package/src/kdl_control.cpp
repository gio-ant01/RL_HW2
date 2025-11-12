#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;
    Eigen::VectorXd ddqd = _ddqd.data;
    
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::computeJointLimitGradient(double _lambda)
{
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::MatrixXd limits = robot_->getJntLimits();
    int n = robot_->getNrJnts();
    Eigen::VectorXd gradient(n);
    
    for (int i = 0; i < n; i++)
    {
        double qi = q(i);
        double qi_min = limits(i,0);
        double qi_max = limits(i,1);
        double range = qi_max - qi_min;
        double dist_from_max = qi_max - qi;
        double dist_from_min = qi - qi_min;
        
        if (dist_from_max < 0.01) dist_from_max = 0.01;
        if (dist_from_min < 0.01) dist_from_min = 0.01;
        
        double numerator = range * range;
        double denominator = dist_from_max * dist_from_min;
        gradient(i) = (1.0 / _lambda) * numerator * (2.0 * qi - qi_max - qi_min) / (denominator * denominator);
    }
    
    return gradient;
}

Eigen::VectorXd KDLController::velocity_ctrl_null(const Eigen::Vector3d &_pos_d,
                                                   const Eigen::Vector3d &_pos_e,
                                                   double _Kp,
                                                   double _lambda)
{
    Eigen::MatrixXd J = robot_->getEEJacobian().data.block(0, 0, 3, robot_->getNrJnts());
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    Eigen::Vector3d e_p = _pos_d - _pos_e;
    Eigen::VectorXd q_dot_primary = J_pinv * (_Kp * e_p);
    Eigen::VectorXd q_dot_0 = computeJointLimitGradient(_lambda);
    
    int n = robot_->getNrJnts();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd null_projector = I - (J_pinv * J);
    Eigen::VectorXd q_dot_secondary = null_projector * q_dot_0;
    Eigen::VectorXd q_dot = q_dot_primary + q_dot_secondary;
    
    return q_dot;
}

// Skew-symmetric matrix operator
Eigen::Matrix3d KDLController::skew(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

// Compute interaction matrix L(s)
Eigen::MatrixXd KDLController::computeInteractionMatrix(const Eigen::Vector3d &_c_Po,
                                                        const Eigen::Matrix3d &_R_c)
{
    // Compute norm
    double norm_c_Po = _c_Po.norm();
    if (norm_c_Po < 1e-6) {
        norm_c_Po = 1e-6;
    }

    // Unit vector s
    Eigen::Vector3d s = _c_Po / norm_c_Po;
    
    // (I - s*s^T)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I_minus_ssT = I - (s * s.transpose());
    
    // S(s) - skew-symmetric
    Eigen::Matrix3d S_s = skew(s);
    
    // Matrix R: transforms FROM world TO camera frame
    Eigen::Matrix<double, 6, 6> R;
    R.setZero();
    R.block<3,3>(0,0) = _R_c.transpose();
    R.block<3,3>(3,3) = _R_c.transpose();
    
    // L(s) in world frame
    Eigen::Matrix<double, 3, 6> L_world;
    L_world.block<3,3>(0,0) = -(1.0/norm_c_Po) * I_minus_ssT;
    L_world.block<3,3>(0,3) = S_s;
    
    // Final L(s) = L_world * R
    Eigen::MatrixXd L_s = L_world * R;
    
    return L_s;  // 3x6
}

// Vision-based controller 
Eigen::VectorXd KDLController::vision_ctrl(const Eigen::Vector3d &_c_Po,
                                          const Eigen::Matrix3d &_R_c,
                                          const Eigen::MatrixXd &_J_cam,
                                          const Eigen::Vector3d &_s_d,
                                          double _K,
                                          double _lambda)
{
    
    double norm_c_Po = _c_Po.norm();
    if (norm_c_Po < 1e-6) {
        int n = robot_->getNrJnts();
        return Eigen::VectorXd::Zero(n);
    }
    
    // Compute interaction matrix L(s)
    Eigen::MatrixXd L_s = computeInteractionMatrix(_c_Po, _R_c);  // 3x6
    
    // Compute L(s) * J_cam
    Eigen::MatrixXd L_J = L_s * _J_cam;  // 3xn
    
    // Compute pseudoinverse
    Eigen::MatrixXd L_J_pinv = pseudoinverse(L_J);
    
    int n = robot_->getNrJnts();
    
    // PRIMARY TASK: Following homework eq. (3) exactly
    // q̇ = K(L(s)Jc)† s_d  (NOT s_error!)
    Eigen::VectorXd q_dot_primary =_K * (L_J_pinv * _s_d);
    
    // SECONDARY TASK: Joint limit avoidance
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd N = I - (L_J_pinv * L_J);
    
    Eigen::VectorXd q_dot_0 = computeJointLimitGradient(_lambda);
    Eigen::VectorXd q_dot_secondary = N * q_dot_0;
    
    // TOTAL velocity: primary + secondary
    Eigen::VectorXd q_dot = q_dot_primary + q_dot_secondary;
    
    // SAFETY: Limit velocities to avoid saturation/instability
    double max_vel = 0.3;  // Reduced from 0.5 for stability
    for (int i = 0; i < n; i++) {
        if (std::abs(q_dot(i)) > max_vel) {
            q_dot(i) = (q_dot(i) > 0) ? max_vel : -max_vel;
        }
    }
    
    return - q_dot;
}