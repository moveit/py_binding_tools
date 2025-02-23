cmake_minimum_required(VERSION 3.12)
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

add_library(${PROJECT_NAME}
  src/ros_msg_typecasters.cpp
  src/initializer.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}
	PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  SYSTEM PUBLIC ${catkin_INCLUDE_DIRS}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(std_msgs REQUIRED)
  pybind_add_module(${PROJECT_NAME}_test test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test PRIVATE ${PROJECT_NAME} ${std_msgs_TARGETS})
  install(TARGETS ${PROJECT_NAME}_test LIBRARY DESTINATION "${CATKIN_GLOBAL_PYTHON_DESTINATION}")

  add_rostest(test/basic.test)
endif()
