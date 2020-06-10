#include "../include/robot.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "Bullet3Common/b3HashMap.h"

#include <iostream>

struct RobotInternalData
{
	int robot_id;
	uint32_t robot_dof;

	RobotInternalData()
		: robot_id(-1), robot_dof(0)
	{
	}

	b3HashMap<b3HashString, int> joint_name_to_id;
};

Robot::Robot()
{
	data_ = new RobotInternalData();
}

Robot::~Robot()
{
	delete data_;
}

unsigned int Robot::Dof() const
{
	return data_->robot_dof;
}

void Robot::SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredAngle, const double& maxTorque, const double& kp, const double& kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(data_->robot_id, jointIdx, controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredTorque)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = desiredTorque;
	sim->setJointMotorControl(data_->robot_id, jointIdx, controlArgs);
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, double& q, double& dq)
{
	struct b3JointSensorState state;
	sim->getJointState(data_->robot_id, jointIdx, &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	// release all motors
	int numJoints = sim->getNumJoints(data_->robot_id);
	for (int i = 0; i < numJoints; i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(data_->robot_id, i, controlArgs);
	}
}

int Robot::Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const btVector3& startPos, const btQuaternion& startOrn)
{
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;
	args.m_forceOverrideFixedBase = true;
	args.m_flags |= URDF_USE_INERTIA_FROM_FILE;

	data_->robot_id = sim->loadURDF(urdf_path, args);
	data_->robot_dof = sim->getNumJoints(data_->robot_id);
	for (int i = 0; i < data_->robot_dof; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(data_->robot_id, i, &jointInfo);
		if (jointInfo.m_jointName[0])
		{
			std::cout << "joint " << i << " maps to " << jointInfo.m_jointName << std::endl;
			data_->joint_name_to_id.insert(jointInfo.m_jointName, i);
		}
	}

	ResetPose(sim);

	return data_->robot_id;
}
