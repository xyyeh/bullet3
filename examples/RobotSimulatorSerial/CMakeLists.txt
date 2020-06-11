
include_directories( 
		${BULLET_PHYSICS_SOURCE_DIR}/src
		${BULLET_PHYSICS_SOURCE_DIR}/examples
		${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
  )
  
set(SRCS
	main.cpp
	bullet_api/b3RobotSimulatorClientAPI.cpp
	external/src/robot.cpp
	external/src/shm_sem.cpp
	external/src/tinyxml2.cpp
	../ExampleBrowser/InProcessExampleBrowser.cpp
	../SharedMemory/GraphicsServerExample.cpp
	../SharedMemory/RemoteGUIHelper.cpp
	../SharedMemory/PhysicsServerExample.cpp
	../SharedMemory/PhysicsServerExampleBullet2.cpp
	../SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp		
)	

add_executable(RobotSimulatorSerial ${SRCS})

target_link_libraries(RobotSimulatorSerial pthread rt BulletRobotics BulletExampleBrowserLib BulletFileLoader BulletWorldImporter BulletSoftBody BulletDynamics BulletCollision BulletInverseDynamicsUtils BulletInverseDynamics LinearMath OpenGLWindow gwen Bullet3Common)

