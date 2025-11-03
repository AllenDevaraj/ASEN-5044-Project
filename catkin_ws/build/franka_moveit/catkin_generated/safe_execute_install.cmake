execute_process(COMMAND "/home/perception/catkin_ws/build/franka_moveit/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/perception/catkin_ws/build/franka_moveit/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
