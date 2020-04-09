#include "Robot.h"
#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "Bullet3Common/b3HashMap.h"

#include <iostream>

struct RobotInternalData
{
	int m_RobotId;
	uint32_t m_RobotDof;

	RobotInternalData()
		: m_RobotId(-1), m_RobotDof(0)
	{
	}

	b3HashMap<b3HashString, int> m_jointNameToId;
};

Robot::Robot()
{
	m_data = new RobotInternalData();
}

Robot::~Robot()
{
	delete m_data;
}

unsigned int Robot::Dof() const
{
	return m_data->m_RobotDof;
}

void Robot::SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredAngle, const double& maxTorque, const double& kp, const double& kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = maxTorque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = desiredAngle;
	sim->setJointMotorControl(m_data->m_RobotId, jointIdx, controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, const double& desiredTorque)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = desiredTorque;
	sim->setJointMotorControl(m_data->m_RobotId, jointIdx, controlArgs);
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& jointIdx, double& q, double& dq)
{
	struct b3JointSensorState state;
	sim->getJointState(m_data->m_RobotId, jointIdx, &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	//release all motors
	int numJoints = sim->getNumJoints(m_data->m_RobotId);
	for (int i = 0; i < numJoints; i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(m_data->m_RobotId, i, controlArgs);
	}
}

int Robot::Setup(class b3RobotSimulatorClientAPI_NoGUI* sim, const std::string& urdf_path, const btVector3& startPos, const btQuaternion& startOrn)
{
	b3RobotSimulatorLoadUrdfFileArgs args;
	args.m_startPosition = startPos;
	args.m_startOrientation = startOrn;

	m_data->m_RobotId = sim->loadURDF(urdf_path, args);

	m_data->m_RobotDof = sim->getNumJoints(m_data->m_RobotId);
	for (int i = 0; i < m_data->m_RobotDof; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(m_data->m_RobotId, i, &jointInfo);
		if (jointInfo.m_jointName[0])
		{
			std::cout << "joint " << i << " maps to " << jointInfo.m_jointName << std::endl;
			m_data->m_jointNameToId.insert(jointInfo.m_jointName, i);
		}
	}

	ResetPose(sim);

	return m_data->m_RobotId;
}
