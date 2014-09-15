include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PC_EIGEN_INCLUDE_DIRS}
    ${PC_MINGXINGCONTROLLER_INCLUDE_DIRS}
    ${PC_KUKAKDL_INCLUDE_DIRS}
)

link_directories(
    ${PC_KUKAKDL_LIBRARY_DIRS}
    ${PC_MINGXINGCONTROLLER_LIBRARY_DIRS}
)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libKukaModelFromFri-<target>.so
# and installs in the directory lib/orocos/
#	
orocos_library(ModelKukaFixed kukafixed.cpp)
orocos_component(KukaMingXingController kukaMingXingController-rtnetcomponent.cpp) # ...you may add multiple source files.
target_link_libraries(ModelKukaFixed ${PC_MINGXINGCONTROLLER_LIBRARIES})
target_link_libraries(KukaMingXingController ModelKukaFixed ${PC_KUKAKDL_LIBRARIES} ${PC_MINGXINGCONTROLLER_LIBRARIES})
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/kuka_send_joint_positions/ directory
#
orocos_install_headers(kukaMingXingController-rtnetcomponent.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

