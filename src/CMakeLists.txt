include_directories(${PROJECT_SOURCE_DIR}/include)
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
orocos_component(KukaMingXingController kukaMingXingController-rtnetcomponent.cpp) # ...you may add multiple source files
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

