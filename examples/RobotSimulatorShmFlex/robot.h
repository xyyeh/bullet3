#pragma once

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "flexible_joint.h"
#include <string>

class Robot
{
private:
	/** internal data */
	struct RobotInternalData* data_;

	/**
   * @brief Resets the pose of the robot
   * @param[in] sim Pointer to simulation interface
   */
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
   * @param[in] sim Pointer to simulation interface`
   * @param[in] urdf_path URDF file location
   * @param[in] start_pos Starting position (defaults to 0,0,0)
   * @param[in] start_ori Starting orientation (defaults to 0,0,0,1)
   * @return Id of the robot
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
   * @brief Returns motor position
   * @param[in] joint_idx Joint index
   * @return Motor position
   */
	double MotorPosition(const int& joint_idx) const;

	/**
   * @brief Returns motor velocity
   * @param[in] joint_idx Joint index
   * @return Motor velocity
   */
	double MotorVelocity(const int& joint_idx) const;

	/**
   * @brief Returns link position
   * @param[in] joint_idx Joint index
   * @return Link position
   */
	double LinkPosition(const int& joint_idx) const;

	/**
   * @brief Returns link velocity
   * @param[in] joint_idx Joint index
   * @return Link velocity
   */
	double LinkVelocity(const int& joint_idx) const;

	/**
   * @brief Returns motor position
   * @param[in] joint_idx Joint index
   * @return Link torque
   */
	double LinkTorque(const int& joint_idx) const;

	/**
   * @brief Returns motor position
   * @param[in] joint_idx Joint index
   * @return Link torque derivative
   */
	double LinkDtorque(const int& joint_idx) const;

	/**
   * @brief Returns motor position
   * @param[in] joint_idx Joint index
   * @param[in] tau_cmd Commanded motor torque
   */
	void SetJointTorque(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_cmd);

	/**
   * @brief Returns motor position
   * @param[in] joint_idx Joint index
   * @param[in] dt Step time
   */
	void StepJointDynamics(const int& joint_idx, const double& dt);
};
