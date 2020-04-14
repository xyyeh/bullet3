#include "b3RobotSimulatorClientAPI.h"

#include <string.h>
#include <iostream>
#include <assert.h>
#include <sys/mman.h>
#include <sys/shm.h>

#include "robot.h"
#include "contact_sensor.h"
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
	Robot o2;
	int o2_id = o2.Setup(sim, "./o2/o2.urdf", btVector3(0, 0, 1.5));  //, btQuaternion(0, 0, 0.38268343, 0.92387953));
	std::cout << "loaded dof = " << o2.LoadedDof() << std::endl;
	std::cout << "active dof = " << o2.ActiveDof() << std::endl;

	// additional setup
	for (int i = -1; i < o2.LoadedDof(); i++)
	{
		sim->setCollisionFilterGroupMask(o2_id, i, 0, 0);
	}

	// setup dynamics
	struct b3RobotSimulatorChangeDynamicsArgs dynArgs;
	dynArgs.m_linearDamping = 0;
	dynArgs.m_angularDamping = 0;
	sim->changeDynamics(o2_id, 0, dynArgs);

	// simulation paramters
	sim->setRealTimeSimulation(false);

	// variables
	std::vector<double> q(o2.ActiveDof());
	std::vector<double> dq(o2.ActiveDof());
	std::vector<double> tau_d(o2.ActiveDof());
	std::vector<double> q_d(o2.ActiveDof());

	// clang-format off
	q_d[6] = q_d[13] = -0.4;
	q_d[7] = 1; q_d[14] = -q_d[7];
	q_d[8] = 0.4; q_d[15] = -q_d[8];
  q_d[9] = q_d[16] = 1.5;
  q_d[10] = 1; q_d[17] = -q_d[10];
  q_d[11] = q_d[18] = -0.4;
  q_d[12] = 0.4; q_d[19] = -q_d[12];
	// clang-format on

	// start loop
	double sim_time = 0;
	double prev_time = 0;

	// setup additional stuffs
	int block_id = sim->loadURDF("cube.urdf");
	btVector3 pos(0, 2, 3);
	btQuaternion ori(0, 0, 0, 1);
	sim->resetBasePositionAndOrientation(block_id, pos, ori);

	// contact sensor
	ContactSensor sensor;
	sensor.Setup(block_id, plane_id);

	double forces[6];

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

		o2.JointStates(sim, q, dq);

		// shared_memory.Lock();
		// // read states and send torques
		// for (uint32_t i = 0; i < o2.Dof(); i++)
		// {
		// 	o2.JointState(sim, i, ptr->q[i], ptr->dq[i]);
		// 	o2.SetDesiredTau(sim, i, ptr->tau[i]);
		// }
		// shared_memory.Unlock();

		o2.SetDesiredTaus(sim, tau_d);

		for (int i = 0; i < q.size(); i++)
		{
			// desired torque values
			if (i < 6)
			{
				tau_d[i] = 16000 * (q_d[i] - q[i]) + 800 * (0 - dq[i]);
			}
			else
			{
				tau_d[i] = 100 * (q_d[i] - q[i]) + 4 * (0 - dq[i]);
			}
		}

		sensor.Wrench(sim, forces, true);

		sim->stepSimulation();

		if ((sim_time - prev_time) > 0.5)
		{
			prev_time = sim_time;

			for (int i = 0; i < 6; i++)
			{
				printf("%.3f\t", forces[i]);
			}
			printf("\n");
		}

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

		// calculate next shot
		t.tv_nsec += (kFixedTimeStep * kSecToNanosec);
		while (t.tv_nsec >= kSecToNanosec)
		{
			t.tv_nsec -= kSecToNanosec;
			t.tv_sec++;
		}
	}

	std::cout << "Detach shared memory" << std::endl;
	shared_memory.Detach();

	std::cout << "Disconnect sim" << std::endl;
	sim->disconnect();

	std::cout << "Delete sim and exit" << std::endl;
	delete sim;
}
