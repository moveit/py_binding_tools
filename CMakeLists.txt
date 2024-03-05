cmake_minimum_required(VERSION 3.0.2)
project(py_binding_tools)

find_package(catkin REQUIRED
COMPONENTS
	geometry_msgs
	pybind11_catkin
	roscpp
)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/ros_msg_typecasters.cpp
  src/roscpp_initializer.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#catkin_lint cannot detect target declarations in functions, here in pybind11_add_module
#catkin_lint: ignore undefined_target

pybind_add_module(${PROJECT_NAME}_module src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME}_module PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_module PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_module
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
