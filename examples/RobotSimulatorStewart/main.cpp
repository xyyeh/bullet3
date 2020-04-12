#include "b3RobotSimulatorClientAPI.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>

#define ASSERT_EQ(a, b) assert((a) == (b));
#include "robot.h"
#include "shm_sem.h"

const unsigned int kSecToNanosec = 1e9;

struct RobotData
{
	double tau[7];
	double q[7];
	double dq[7];
};

int main(int argc, char* argv[])
{
	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	if (!sim->connect(eCONNECT_GUI))
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
	sim->configureDebugVisualizer(COV_ENABLE_SHADOWS, 1);

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
	Robot stewart;
	int robot_id = stewart.Setup(sim, "./stewart/stewart_platform.urdf", btVector3(0, 0, 0.5));
	int robot_base_id = sim->loadURDF("./stewart/stewart_base.urdf");

	// setup dynamics
	struct b3RobotSimulatorChangeDynamicsArgs dyn_args;
	dyn_args.m_linearDamping = 0;
	dyn_args.m_angularDamping = 0;
	sim->changeDynamics(robot_id, 0, dyn_args);

	// additional setup
	for (int i = -1; i < (4 * 6); i++)
	{
		sim->setCollisionFilterGroupMask(robot_id, i, 0, 0);
	}

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// start loop
	double sim_time = 0;
	double tau[6], q[6], dq[6];
	double home[6] = {-0.555272, -0.555272, -0.555272, -0.555272, -0.555272, -0.555272};
	double prev_time = 0;
	double wave[6];

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

		stewart.JointStates(sim, q, dq);
		for (unsigned int i = 0; i < 6; i++)
		{
			wave[i] = sin(2 * M_PI * 0.1 * sim_time) * 0.1 * sin((2 * M_PI * 1 * sim_time) + (2 * M_PI) / 6.0 * i);
			tau[i] = 400 * (wave[i] + home[i] - q[i]) + 40 * (0 - dq[i]);
		}
		stewart.SetDesiredTaus(sim, tau);

		// shared_memory.Lock();
		// read states and send torques
		// for (uint32_t i = 0; i < stewart.Dof(); i++)
		// {
		// 	stewart.JointState(sim, 3, q[0], dq[0]);
		// 	stewart.SetDesiredTau(sim, i, ptr->tau[i]);
		// }
		// shared_memory.Unlock();

		sim->stepSimulation();

		// if ((sim_time - prev_time) > 0.25)
		// {
		// 	prev_time = sim_time;
		// 	for (int i = 0; i < 6; i++)
		// 	{
		// 		printf("%.3f\t", q[i]);
		// 	}
		// 	printf("\n");
		// }

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

		// calculate next shot
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
