
include_directories( RobotSimulatorShmFlex
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
  )
  
set(SRCS
  main.cpp
	b3RobotSimulatorClientAPI.cpp
	robot.cpp
  shm_sem.cpp
  flexible_joint.cpp
	../ExampleBrowser/InProcessExampleBrowser.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
	../SharedMemory/PhysicsServerExample.cpp
	../SharedMemory/PhysicsServerExampleBullet2.cpp
	../SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	

add_executable(RobotSimulatorShmFlex ${SRCS})

target_link_libraries(RobotSimulatorShmFlex pthread rt BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)

