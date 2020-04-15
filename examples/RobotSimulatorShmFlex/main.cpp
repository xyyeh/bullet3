#include "b3RobotSimulatorClientAPI.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <vector>

#include "robot.h"
#include "shm_sem.h"

const unsigned int kSecToNanosec = 1e9;

struct RobotPath
{
#define kLineNum (10)

	b3RobotSimulatorAddUserDebugLineArgs args[kLineNum];
	btVector3 point[kLineNum];
	int id[kLineNum] = {-10, -10, -10, -10, -10, -10, -10, -10, -10, -10};

	int prev_id = 0;
	int curr_id = 0;
};

#define DOF (2)
struct RobotData
{
	double tau[DOF];
	double q[DOF];
	double dq[DOF];
};

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);

	if (!isConnected)
	{
		std::cout << "Cannot connect" << std::endl;
		return -1;
	}

	// shared memory
	ShmSemaphore shared_memory("/physics_shm");
	shared_memory.Create(sizeof(struct RobotData));
	shared_memory.Attach();
	RobotData* ptr = static_cast<RobotData*>(shared_memory.Data());

	// setup visualizer
	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 0);

	// setup timeout
	sim->setTimeOut(10);

	// syncBodies is only needed when connecting to an existing physics server that has already some bodies
	sim->syncBodies();

	// setup fixed time step
	btScalar kFixedTimeStep = 1.0 / 1000.0;
	sim->setTimeStep(kFixedTimeStep);

	// setup gravity
	sim->setGravity(btVector3(0, 0, -9.81));

	// setup world
	int plane_id = sim->loadURDF("plane.urdf");

	// setup robot
	Robot arm;
	int robot_id = arm.Setup(sim, "./panda/two_joints.urdf", btVector3(0, 0, 0.0000));

	// disable collisions
	for (int i = -1; i < arm.Dof(); i++)
	{
		sim->setCollisionFilterGroupMask(robot_id, i, 0, 0);
	}
	sim->setCollisionFilterGroupMask(plane_id, -1, 0, 0);

	// setup dynamics
	struct b3RobotSimulatorChangeDynamicsArgs dynArgs;
	dynArgs.m_linearDamping = 0;
	dynArgs.m_angularDamping = 0;
	sim->changeDynamics(robot_id, 0, dynArgs);

	// setup additional stuffs
	// int blockId = sim->loadURDF("cube.urdf");
	// btVector3 pos(0, 0, 3);
	// btQuaternion ori(0, 0, 0, 1);
	// sim->resetBasePositionAndOrientation(blockId, pos, ori);

	// path
	RobotPath path;

	// frame
	b3RobotSimulatorAddUserDebugLineArgs frame_args;
	btVector3 start(0, 0, 0);
	btVector3 end_x(1, 0, 0);
	btVector3 end_z(0, 0, 1);
	frame_args.m_lineWidth = 5;
	frame_args.m_colorRGB[0] = 1;
	frame_args.m_colorRGB[1] = 0;
	frame_args.m_colorRGB[2] = 0;
	int id_x = sim->addUserDebugLine(start, end_x, frame_args);
	frame_args.m_colorRGB[0] = 0;
	frame_args.m_colorRGB[1] = 0;
	frame_args.m_colorRGB[2] = 1;
	int id_z = sim->addUserDebugLine(start, end_z, frame_args);

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// start loop
	double sim_time = 0;
	double tau[7], q[7], dq[7];
	double q_d[7] = {0, 0.3, 0, -0.3, 0, 1.5, 0};
	double prev_time = 0;

	// start looping after one second
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	t.tv_sec++;
	while (1)
	{
		if (!sim->canSubmitCommand())
		{
			std::cout << "//////////////////// WARNING ////////////////////" << std::endl;
		}

		sim_time += kFixedTimeStep;

		// read states and send torques
		// shared_memory.Lock();
		for (uint32_t i = 0; i < arm.Dof(); i++)
		{
			double q, dq;
			arm.JointState(sim, i, q, dq);
			double cmd = 0 * (0 - q) - 4 * dq;
			ptr->tau[i] = arm.StepJointDynamics(sim, i, cmd, q, dq);
			arm.SetDesiredTau(sim, i, ptr->tau[i]);
		}
		// shared_memory.Unlock();

		// for (uint32_t i = 0; i < arm.Dof(); i++)
		// 	arm.SetDesiredTau(sim, i, 0);

		// step simulation
		sim->stepSimulation();

		// b3LinkState link_state;
		// sim->getLinkState(robot_id, 5, 0, 0, &link_state);

		if ((sim_time - prev_time) > 0.25)
		{
			prev_time = sim_time;
			// for (int i = 0; i < 3; i++)
			// {
			// 	printf("%.3f\t", link_state.m_worldPosition[i]);
			// }
			// printf("\n");

			// for (int i = 0; i < 10; i++)
			// {
			// 	printf("%d\t", path.id[i]);
			// }
			// printf("\n");

			// // remove old
			// if (path.id[path.curr_id] != -10)
			// {
			// 	sim->removeUserDebugItem(path.id[path.curr_id]);
			// }
			// // add new
			// path.prev_id = path.curr_id;
			// path.curr_id = (path.prev_id + 1) % kLineNum;
			// path.point[path.curr_id][0] = link_state.m_worldPosition[0];
			// path.point[path.curr_id][1] = link_state.m_worldPosition[1];
			// path.point[path.curr_id][2] = link_state.m_worldPosition[2];
			// path.args[path.curr_id].m_lineWidth = 3;
			// path.id[path.curr_id] = sim->addUserDebugLine(path.point[path.prev_id], path.point[path.curr_id], path.args[path.curr_id]);
		}

		// calculate next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
		t.tv_nsec += (kFixedTimeStep * kSecToNanosec);
		while (t.tv_nsec >= kSecToNanosec)
		{
			t.tv_nsec -= kSecToNanosec;
			t.tv_sec++;
		}
	}

	std::cout << "Disconnect sim" << std::endl;

	sim->disconnect();

	std::cout << "Delete sim and exit" << std::endl;
	delete sim;
}
