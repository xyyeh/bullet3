#include "../include/robot.h"
#include "../include/tinyxml2.h"

#include "b3RobotSimulatorClientAPI_NoGUI.h"
#include "Bullet3Common/b3HashMap.h"

#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct RobotInternalData
{
	int robot_id;
	uint32_t robot_dof;

	RobotInternalData()
		: robot_id(-1), robot_dof(0)
	{
	}

	b3HashMap<b3HashString, int> joint_name_to_id;

	std::vector<double> kp;
	std::vector<double> kd;
	std::vector<double> home;
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

void Robot::SetDesiredQ(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& q_des, const double& max_torque, const double& kp, const double& kd)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_POSITION_VELOCITY_PD);
	controlArgs.m_maxTorqueValue = max_torque;
	controlArgs.m_kd = kd;
	controlArgs.m_kp = kp;
	controlArgs.m_targetPosition = q_des;
	sim->setJointMotorControl(data_->robot_id, joint_idx, controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_des)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = tau_des;
	sim->setJointMotorControl(data_->robot_id, joint_idx, controlArgs);
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq)
{
	struct b3JointSensorState state;
	sim->getJointState(data_->robot_id, joint_idx, &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	// set to home and release all motors
	int numJoints = sim->getNumJoints(data_->robot_id);
	for (int i = 0; i < numJoints; i++)
	{
		// sim->resetJointState(data_->robot_id, i, data_->home[i]);

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

	// get additional information from modified urdf if available
	auto ReadAttribute = [](tinyxml2::XMLElement* dom, const std::string& attribute) {
		double data = 0;
		if (dom)
		{
			const char* attr = dom->Attribute(attribute.c_str());
			if (attr)
			{
				std::stringstream ss;
				ss.imbue(std::locale::classic());
				ss << attr;

				if (ss.good())
				{
					ss >> data;
				}
			}
		}
		return data;
	};

	std::ifstream file(urdf_path);
	if (file.is_open())
	{
		std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

		tinyxml2::XMLDocument doc;
		doc.Parse(contents.c_str());

		tinyxml2::XMLElement* joint_tag = doc.FirstChildElement("robot")->FirstChildElement("joint");

		while (joint_tag)
		{
			tinyxml2::XMLElement* ctrl_tag = joint_tag->FirstChildElement("safety_controller");
			if (ctrl_tag)
			{
				data_->kp.push_back(ReadAttribute(ctrl_tag, "k_position"));
				data_->kd.push_back(ReadAttribute(ctrl_tag, "k_velocity"));
				data_->home.push_back(ReadAttribute(ctrl_tag, "home"));
			}

			joint_tag = joint_tag->NextSiblingElement("joint");
		}
	}
	else
	{
		std::cout << "urdf file not found for further parsing" << std::endl;
	}

	return data_->robot_id;
}

double Robot::Kp(const int& joint_idx) const
{
	return data_->kp[joint_idx];
}

double Robot::Kd(const int& joint_idx) const
{
	return data_->kd[joint_idx];
}

double Robot::Home(const int& joint_idx) const
{
	return data_->home[joint_idx];
}