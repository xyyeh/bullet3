#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"

#include <string>

class Robot
{
	struct RobotInternalData* data_;
	void ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim);

public:
	Robot();
	virtual ~Robot();

	/**
	 * @brief Returns dof of robot
	 * @return dof of the robot
	 */
	unsigned int Dof() const;

	/**
	 * @brief Sets up the robot in the simulator using an urdf
	 * @param[in] urdf_path Filename for urdf
	 * @param[in] starting_pos Starting position
	 * @param[in] starting_ori Starting orientation
	 */
	int Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const class btVector3& starting_pos = btVector3(0, 0, 0), const class btQuaternion& starting_ori = btQuaternion(0, 0, 0, 1));

	/**
	 * @brief Sets the desired joint angle q using bullet's position controller
	 * @param[in] sim Pointer to simulation interface
	 * @param[in] joint_idx Joint index
	 * @param[in] max_torque Maximum joint torque
	 * @param[in] kp Position control stiffness gain
	 * @param[in] kd Position control damping gain
	 */
	void SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& q_des, const double& max_torque = 300, const double& kp = 400, const double& kd = 40);

	/**
	 * @brief Sets the desired joint torque
	 * @param[in] sim Pointer to simulation interface
	 * @param[in] joint_idx Joint index
	 * @param[in] tau_des Joint torque
	 */
	void SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_des);

	/**
	 * @brief Gets the joint state
	 * @param[in] sim Pointer to simulation interface
	 * @param[in] joint_idx Joint index
	 * @param[out] q Joint position
	 * @param[out] dq Joint velocity
	 */
	void JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq);

	/**
	 * @brief Gets the kp of joint
	 * @param[in] joint_idx Joint index
	 * @return Kp of joint
	 */
	double Kp(const int& joint_idx) const;

	/**
	 * @brief Gets the kp of joint
	 * @param[in] joint_idx Joint index
	 * @return Kd of joint
	 */
	double Kd(const int& joint_idx) const;

	/**
	 * @brief Gets the home position of joint
	 * @param[in] joint_idx Joint index
	 * @return Home position of joint
	 */
	double Home(const int& joint_idx) const;
};
