#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

#include <string>

class Robot
{
private:
	struct RobotInternalData* data_;
	void ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

public:
	Robot();
	virtual ~Robot();

	/**
   * @brief Returns the number of dofs for the robot
   * @return Dofs of the robot
   */
	unsigned int Dof() const;

	/**
   * @brief Sets up the simulation interface
   * @param[in] sim Pointer to simulation interface
   * @param[in] urdf_path URDF file location
   * @param[in] start_pos Starting position (defaults to 0,0,0)
   * @param[in] start_ori Starting orientation (defaults to 0,0,0,1)
   */
	int Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const class btVector3& start_pos = btVector3(0, 0, 0), const class btQuaternion& start_ori = btQuaternion(0, 0, 0, 1));

	/**
   * @brief Sets up the desired link angle
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Joint index
   * @param[in] q_d Desired link angle
   * @param[in] tau_max Maximum allowable torque
   * @param[in] kp Position gains (defaults to 400)
   * @param[in] kv Velocity gains (defaults to 40)
   */
	void SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& q_d, const double& tau_max = 300, const double& kp = 400, const double& kd = 40);

	/**
   * @brief Sets up the desired torque
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Joint index
   * @param[in] tau_d Desired torque
   */
	void SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_d);

	/**
   * @brief Sets up the desired torque
   * @param[in] sim Pointer to simulation interface
   * @param[in] joint_idx Joint index
   * @param[out] q Link angle
   * @param[out] dq Link velocity
   */
	void JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq);
};
