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
#include <map>

struct RobotInternalData
{
	int robot_id;
	uint32_t robot_dof;

	RobotInternalData()
		: robot_id(-1), robot_dof(0)
	{
	}

	std::vector<double> kp;
	std::vector<double> kd;
	std::vector<double> home;

	std::map<unsigned int, std::string> active_id_to_joint_name;
	std::map<std::string, unsigned int> joint_name_to_id;
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

	unsigned int actuated_id = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));

	sim->setJointMotorControl(data_->robot_id, actuated_id, controlArgs);
}

void Robot::SetDesiredTau(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, const double& tau_des)
{
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	controlArgs.m_maxTorqueValue = tau_des;

	unsigned int actuated_id = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));

	sim->setJointMotorControl(data_->robot_id, actuated_id, controlArgs);
}

void Robot::JointState(class b3RobotSimulatorClientAPI_NoGUI* sim, const int& joint_idx, double& q, double& dq)
{
	struct b3JointSensorState state;

	unsigned int actuated_id = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));

	sim->getJointState(data_->robot_id, actuated_id, &state);
	q = state.m_jointPosition;
	dq = state.m_jointVelocity;
}

void Robot::ResetPose(class b3RobotSimulatorClientAPI_NoGUI* sim)
{
	// set to home and release all motors
	for (int i = 0; i < sim->getNumJoints(data_->robot_id); i++)
	{
		b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		controlArgs.m_maxTorqueValue = 0;
		sim->setJointMotorControl(data_->robot_id, i, controlArgs);
	}

	// only update active dofs
	for (unsigned int i = 0; i < data_->robot_dof; i++)
	{
		int idx = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(i));
		sim->resetJointState(data_->robot_id, idx, data_->home[idx]);
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
	data_->robot_dof = 0;
	for (unsigned int i = 0; i < sim->getNumJoints(data_->robot_id); i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(data_->robot_id, i, &jointInfo);

		if (jointInfo.m_jointName[0])
		{
			std::string name(jointInfo.m_jointName);
			std::size_t found = name.find("fixed");

			if (found == std::string::npos)
			{
				data_->active_id_to_joint_name.insert(std::pair<unsigned int, std::string>(data_->robot_dof, name));
				std::cout << "active_dof " << data_->robot_dof << " maps to " << jointInfo.m_jointName << std::endl;

				data_->robot_dof++;
			}

			data_->joint_name_to_id.insert(std::pair<std::string, unsigned int>(std::string(jointInfo.m_jointName), i));
		}
	}
	std::cout << "actuated dof = " << data_->robot_dof << std::endl;

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

	unsigned int num_dof = sim->getNumJoints(data_->robot_id);
	data_->kp.resize(num_dof);
	data_->kd.resize(num_dof);
	data_->home.resize(num_dof);

	if (file.is_open())
	{
		std::string contents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		tinyxml2::XMLDocument doc;
		doc.Parse(contents.c_str());

		tinyxml2::XMLElement* joint_tag = doc.FirstChildElement("robot")->FirstChildElement("joint");
		while (joint_tag)
		{
			std::string joint_name = joint_tag->Attribute("name");
			unsigned int id = data_->joint_name_to_id.at(joint_name);
			tinyxml2::XMLElement* ctrl_tag = joint_tag->FirstChildElement("safety_controller");
			if (ctrl_tag)
			{
				data_->kp[id] = ReadAttribute(ctrl_tag, "k_position");
				data_->kd[id] = ReadAttribute(ctrl_tag, "k_velocity");
				data_->home[id] = ReadAttribute(ctrl_tag, "home");
			}

			joint_tag = joint_tag->NextSiblingElement("joint");
		}
	}
	else
	{
		std::cout << "urdf file not found for further parsing" << std::endl;
	}

	ResetPose(sim);

	return data_->robot_id;
}

double Robot::Kp(const int& joint_idx) const
{
	unsigned int idx = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));
	return data_->kp[idx];
}

double Robot::Kd(const int& joint_idx) const
{
	unsigned int idx = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));
	return data_->kd[idx];
}

double Robot::Home(const int& joint_idx) const
{
	unsigned int idx = data_->joint_name_to_id.at(data_->active_id_to_joint_name.at(joint_idx));
	return data_->home[idx];
}