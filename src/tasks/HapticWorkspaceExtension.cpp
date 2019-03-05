/*
 * HapticWorkspaceExtension.cpp
 *
 *      Author: Margot
 */

#include "HapticWorkspaceExtension.h"

#include <stdexcept>


namespace Sai2Primitives
{


HapticWorkspaceExtension::HapticWorkspaceExtension(double Rmax_dev, double Rmax_env,
				double Thetamax_dev, double Thetamax_env)
{
	// Initialization of the controller parameters
	_Rmax_dev=Rmax_dev;
	_Rmax_env=Rmax_env;
	_Thetamax_dev=Thetamax_dev;
	_Thetamax_env=Thetamax_env;

	// Default home position (can be change with setDeviceCenter())
	_HomePos_op.setZero();
	_HomeRot_op.setIdentity();

	// Initialize workspace center of the controlled robot
	_centerPos_rob.setZero(); 
	_centerRot_rob.setIdentity();

	// Default drift force percentage (can be change with setForceNoticeableDiff())
	_Fdrift_perc=10.0/100.0;


	_Ks=1.0;
	_KsR=1.0;
	_vel_rot_max=0.00001;
	_vel_trans_max=0.00001;
	
	_Fdrift.setZero(6);
	_vel_drift_rot.setZero(3);
	_vel_drift_trans.setZero(3);

	_first_iteration = true; // To initialize the timer
}


void HapticWorkspaceExtension::computeHapticCommands(Eigen::VectorXd& Fop_des,
					Eigen::Vector3d& pos_rob,
					Eigen::Matrix3d& rot_rob,
					const Eigen::Vector3d pos_op, 
		            const Eigen::Matrix3d rot_op,
			    	const Eigen::VectorXd vel_op,
			    	const Eigen::VectorXd Fop_task,
			    	const Eigen::MatrixXd Lambda)
{
	// get time since last call
	_t_curr = std::chrono::high_resolution_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::high_resolution_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;


	//Eigen::Affine3d position = Eigen::Affine3d::Identity();
	//position.linear() = _rot_op;
	//position.translation() = _pos_op;

	//Read velocities
	Eigen::Vector3d _vel_op_trans = vel_op.head(3);
	Eigen::Vector3d _vel_op_rot = vel_op.tail(3);

	// Update the maximum velocities for the task
	if (_vel_op_rot.norm()>=_vel_rot_max)	
	{
		_vel_rot_max = _vel_op_rot.norm();

        }

	if (_vel_op_trans.norm()>=_vel_trans_max)	
	{
		_vel_trans_max = _vel_op_trans.norm();

        }

//////////Evaluation of the drift velocities/////////////////////
	//// Translational drift ////
	Eigen::Vector3d _pos_rel;
	_pos_rel = pos_op-_HomePos_op;
	_vel_drift_trans = -_vel_op_trans.norm()*_pos_rel/(_Rmax_dev*_vel_trans_max);
	//// Rotational drift ////
	Eigen::Matrix3d _rot_rel = (_HomeRot_op.transpose())*rot_op; // Rotation with respect with home orientation
	Eigen::AngleAxisd _rot_rel_ang = Eigen::AngleAxisd(_rot_rel);
	_vel_drift_rot=-(_vel_op_rot.norm()*_rot_rel_ang.angle()*_rot_rel_ang.axis())/(_Thetamax_dev*_vel_rot_max);

////////// Evaluation of the scaling factors /////////////////////	
	//// Translational scaling ////
	_Ks = 1.0 + _pos_rel.norm()*(_Rmax_env/_Rmax_dev-1.0)/_Rmax_dev;
	//// Rotational scaling ////
	_KsR = 1.0 + _rot_rel_ang.angle()*(_Thetamax_env/_Thetamax_dev-1.0)/_Thetamax_dev;

	Eigen::MatrixXd scaling_factor = Eigen::MatrixXd::Identity(6,6);
		scaling_factor << 1/_Ks, 0.0, 0.0, 0.0, 0.0, 0.0,
						  0.0, 1/_Ks, 0.0, 0.0, 0.0, 0.0, 
						  0.0, 0.0, 1/_Ks, 0.0, 0.0, 0.0,
						  0.0, 0.0, 0.0, 1/_KsR, 0.0, 0.0, 
						  0.0, 0.0, 0.0, 0.0, 1/_KsR, 0.0,
						  0.0, 0.0, 0.0, 0.0, 0.0, 1/_KsR;
	Eigen::VectorXd Fop_task_scal = Eigen::VectorXd::Zero(6);
	Fop_task_scal = scaling_factor*Fop_task;
	
//////////Evaluation of the drift force /////////////////////
	Eigen::MatrixXd _Kv = _Fdrift_perc*(Fop_task_scal.asDiagonal()); // Definition of the velocity gain from task feedback
	Eigen::VectorXd _vel_drift(6);
	_vel_drift << _vel_drift_trans,_vel_drift_rot; // Drift velocity vector concatenation
	_Fdrift = _Kv*_vel_drift; //Drift
	//_Fdrift = Lambda*_Fdrift_0; //Drift force weigthed through the device inertia matrix
////////// Desired cartesian force to apply to the haptic device /////////////////////
	Fop_des = _Fdrift + Fop_task_scal;

////////// Computation of the desired position for the controlled robot after drift of the device /////////////////////
	//// Estimated drift velocity considering drift force and estimated human+device mass matrix ////
	Eigen::VectorXd _vel_drift_est = _t_diff.count()*Lambda.inverse()*_Fdrift;
	Eigen::Vector3d _vel_drift_est_trans = _vel_drift_est.head(3);
	Eigen::Vector3d _vel_drift_est_rot = _vel_drift_est.tail(3);
	//// Drift of the task workspace ////
	_centerPos_rob = _centerPos_rob - _t_diff.count()*_Ks*_vel_drift_est_trans;
	double _centerRot_angle = -_t_diff.count()*_KsR*_vel_drift_est_rot.norm();
	Eigen::Vector3d _centerRot_axis;
	if (abs(_centerRot_angle) <= 0.00001)
	{
		_centerRot_rob = _centerRot_rob;
	}
	else 
	{ 
		_centerRot_axis = _vel_drift_est_rot/_vel_drift_est_rot.norm();
		Eigen::AngleAxisd _centerRot_angleAxis=Eigen::AngleAxisd(_centerRot_angle,_centerRot_axis);
		_centerRot_rob = _centerRot_rob*_centerRot_angleAxis.toRotationMatrix();
	}
	//// Set position of the controlled robot after drift ////
	pos_rob = _centerPos_rob + _Ks*_pos_rel;
	Eigen::AngleAxisd _Rot_dev_rob = Eigen::AngleAxisd(_KsR*_rot_rel_ang.angle(),_rot_rel_ang.axis());
	rot_rob = _centerRot_rob*(_Rot_dev_rob.toRotationMatrix());
	
	// update previous time
	_t_prev = _t_curr;
}



void HapticWorkspaceExtension::reInitializeTask()
{
	// Initialize workspace center of the controlled robot
	_centerPos_rob.setZero(); 
	_centerRot_rob.setIdentity();

	_Ks=1.0;
	_KsR=1.0;
	_vel_rot_max=0.001;
	_vel_trans_max=0.001;
	
	_Fdrift.setZero(6);
	_vel_drift_rot.setZero(3);
	_vel_drift_trans.setZero(3);

	_first_iteration = true; // To initialize the timer
}


void HapticWorkspaceExtension::setWorkspaceSize(double Rmax_dev, double Rmax_env, double Thetamax_dev, double Thetamax_env)
{
	_Rmax_dev = Rmax_dev;
	_Rmax_env = Rmax_env;
	_Thetamax_dev = Thetamax_dev;
	_Thetamax_env = Thetamax_env;
}


void HapticWorkspaceExtension::setDeviceCenter(const Eigen::Vector3d HomePos_op, const Eigen::Matrix3d HomeRot_op)
{
	_HomePos_op = HomePos_op;
	_HomeRot_op = HomeRot_op;
}


void HapticWorkspaceExtension::setForceNoticeableDiff(double Fdrift_perc)
{
	_Fdrift_perc = Fdrift_perc;
}



} /* namespace Sai2Primitives */

