/*
 * HapticWorkspaceExtension.h
 *
 *      This controller implements the workspace extension in haptic application.
 *	It computes the haptic device force and the controlled robot set position
 *	with respect to the input command of the device, the drift of the 
 *	workspace and the task force feedback. 
 *
 *      Author: Margot
 */

#ifndef SAI2_PRIMITIVES_HAPTICWSEXT_TASK_H_
#define SAI2_PRIMITIVES_HAPTICWSEXT_TASK_H_

#include "Sai2Model.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

namespace Sai2Primitives
{

class HapticWorkspaceExtension
{
public:

	//------------------------------------------------
	// Constructors
	//------------------------------------------------


	/**
	 * @brief Constructor  This constructor creates the workspace extension controller for a haptic application.
	 *
	 * @param Rmax_dev        Radius of the smallest sphere including the haptic device workspace
	 * @param Rmax_env        Radius of the smallest sphere including the task environment
	 * @param Thetamax_dev    Maximum tilt angle of the haptic device
	 * @param Thetamax_env    Maximum tilt angle of the controlled robot for the task
	 *
	 * @param robot      A pointer to a Sai2Model object for the haptic device if defined	
	 */
	HapticWorkspaceExtension(double Rmax_dev, double Rmax_env,
				double Thetamax_dev, double Thetamax_env
			    //Sai2Model::Sai2Model* robot
				);


	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief update the haptic device model (mass matrix) (if model defined)
	 * @details This function updates the inertia matrix (Lambda) of the human user with the haptic device. This function uses the robot model and assumes it has been updated.
	 */
	//virtual void updateDeviceModel();


	/**
	 * @brief Computes the haptic device set force and the controlled robot set position
	 * @details Computes the haptic device force and the controlled robot set position with respect to the haptic device positions/velocities, the drift of the workspace and the task force feedback. The controller takes a Vector3d for definition of the device cartesian position and a Matrix3d for the orientation. A VectorXd describes the device velocity and another the desired cartesian force feedback from the task. The haptic device inertia matrix is given in the cartesian space by a Matrix3d.
	 * 
	 * @param Fop_des    The desired cartesian force to apply to the haptic device (F_drift + Fop_task)
	 * @param pos_rob    The desired position of the controlled robot after extension of the workspace
	 * @param rot_rob    The desired orientation of the controlled robot after extension of the workspace
	 * @param pos_op     The position of the haptic device in its operational space
	 * @param rot_op     The orientation of the haptic device in its operational space
	 * @param vel_op     The velocity of the haptic device in its operational space
	 * @param Fop_task   The desired force feedback from the task interaction
	 * @param Lambda     The haptic device and human user mass matrix in the cartesian space
	 */
	void computeHapticCommands(Eigen::VectorXd& Fop_des,
					Eigen::Vector3d& pos_rob,
					Eigen::Matrix3d& rot_rob,
					const Eigen::Vector3d pos_op, 
		            const Eigen::Matrix3d rot_op = Eigen::Matrix3d::Identity(),
			    	const Eigen::VectorXd vel_op = Eigen::VectorXd::Zero(6),
			    	const Eigen::VectorXd Fop_task = Eigen::VectorXd::Zero(6),
			    	const Eigen::MatrixXd Lambda = Eigen::MatrixXd::Identity(6,6));


	/**
	 * @brief      reinitializes the workspace drift to the workspace origin of the controlled robot, the controller parameters are kept as updated, max velocities and drift force/velocity are set back to zero.
	 */
	void reInitializeTask();


	// -------- Workspace extension related methods --------

	/**
	 * @brief Sets the size of the device workspace and the task environment
	 * @details The size of the device workspace and the task environment are set through the radius of the smallest sphere including each workspace and the maximum tilt angles.
	 * 
	 * @param Rmax_dev        Radius of the smallest sphere including the haptic device workspace
	 * @param Rmax_env        Radius of the smallest sphere including the task environment
	 * @param Thetamax_dev    Maximum tilt angle of the haptic device
	 * @param Thetamax_env    Maximum tilt angle of the controlled robot for the task
	 */
	void setWorkspaceSize(double Rmax_dev, double Rmax_env, double Thetamax_dev, double Thetamax_env);

	/**
	 * @brief Sets the center of the device workspace
	 * @details The haptic device home position and orientation are set through a Vector3d and a Matrix3d
	 * 
	 * @param HomePos_op     The home position of the haptic device in its operational space
	 * @param HomeRot_op     The home orientation of the haptic device in its operational space
	 */
	void setDeviceCenter(const Eigen::Vector3d HomePos_op, 
		            const Eigen::Matrix3d HomeRot_op = Eigen::Matrix3d::Identity());


	/**
	 * @brief Sets the percentage of drift force compared to the task force feedback
	 * @details The level of drift force is set with respect to the task force feedback thanks to the just noticeable difference percentage
	 * 
	 * @param Fdrift_perc     Percentage of drift force with repect to the task force feedback
	 */
	void setForceNoticeableDiff(double Fdrift_perc);


	//------------------------------------------------
	// Attributes
	//------------------------------------------------
	
	// Workspace features
	double _Rmax_dev, _Rmax_env, _Thetamax_dev, _Thetamax_env; // Radius (in meter) and max tilt angles (in degree)
	Eigen::Vector3d _HomePos_op; Eigen::Matrix3d _HomeRot_op; // Haptic device home position and orientation

	// Controller parameters
	double _Fdrift_perc; // Percentage of drift force
	double _Ks, _KsR; // Scaling factors in translation and rotation
	double _vel_rot_max, _vel_trans_max; // Maximal haptic device velocity during the task
	
	//Workspace drift components
	Eigen::VectorXd _Fdrift; // Drift force 	
	Eigen::Vector3d _vel_drift_rot; Eigen::Vector3d _vel_drift_trans; // Drift velocities
	Eigen::Vector3d _centerPos_rob; Eigen::Matrix3d _centerRot_rob; // Drifting workspace center of the controlled robot, position and orientation 

	//Time parameters
	std::chrono::high_resolution_clock::time_point _t_prev;
	std::chrono::high_resolution_clock::time_point _t_curr;
	std::chrono::duration<double> _t_diff;
	bool _first_iteration;
	

};


} /* namespace Sai2Primitives */

/* SAI2_PRIMITIVES_HAPTICWSEXT_TASK_H_ */
#endif
