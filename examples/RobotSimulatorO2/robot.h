#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

#include <string>
#include <vector>

class Robot
{
private:
	/** Internal data */
	struct RobotInternalData* data_;

public:
	Robot();
	virtual ~Robot();

	/**
   * @brief Resets pose of the robot during initialization
   * @param[in] sim Pointer to simulation interface
   */
	void ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

	/**
   * @brief Loaded number of dofs
   * @return loaded dofs including passive and fixed joints
   */
	unsigned int LoadedDof() const;

	/**
   * @brief Active number of dofs
   * @return active dofs that can be controlled
   */
	unsigned int ActiveDof() const;

	/**
   * @brief Sets up robot system
   * @param[in] sim Pointer to simulation interface
   * @param[in] urdf_path Path to urdf file
   * @param[in] start_pos Inital position, defaults to (0, 0, 0)
   * @param[in] start_ori Initial orientation, defaults to (0, 0, 0, 1)
   */
	int Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const class btVector3& start_pos = btVector3(0, 0, 0), const class btQuaternion& start_ori = btQuaternion(0, 0, 0, 1));

	/**
   * @brief Sets desired joint angle
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Index of the joint to be position controlled, see the console printouts
   * @param[in] q_d Desired q
   * @param[in] tau_max Saturation value of the maximum torque, defaults to 300 N or Nm
   * @param[in] kp Kp of the PD controller
   * @param[in] kd Kd of the PD controller
   */
	void SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& q_d, const double& tau_max = 300, const double& kp = 400, const double& kd = 40);

	/**
   * @brief Sets desired torque
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Index of the joint to be position controlled, see the console printouts
   * @param[in] tau_d Desired torque
   */
	void SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_d);

	/**
   * @brief Sets desired torque
   * @param[in] sim Pointer to simulation interface
   * @param[in] tau_d Desired torque
   */
	void SetDesiredTaus(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::vector<double>& tau_d);

	/**
   * @brief State of the joint
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Index of the joint which q and dq is requested, see the console printouts
   * @param[in] q Joint position
   * @param[in] dq Joint velocity
   */
	void JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq);

	/**
   * @brief State of the joints
   * @param[in] sim Pointer to simulation interface
   * @param[in] q Joint position
   * @param[in] dq Joint velocity
   */
	void JointStates(class b3RobotSimulatorClientAPI_NoGUI* sim, std::vector<double>& q, std::vector<double>& dq);
};