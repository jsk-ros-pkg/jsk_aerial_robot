cmake_minimum_required(VERSION 2.8.3)
project(acados)
include(ExternalProject)

#message(WARNING "${CMAKE_INSTALL_PREFIX}")

ExternalProject_Add(
  acados
  GIT_REPOSITORY https://github.com/acados/acados.git
  GIT_TAG fe308a5
  CMAKE_ARGS -DACADOS_WITH_QPOASES=ON  -DACADOS_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_INSTALL ON
  )

# # SUCCESS
#ExternalProject_Add_Step(acados pip_install
#        COMMAND pip install -e <BINARY_DIR>/../acados/interfaces/acados_template
#        DEPENDEES install
#)

# # FAIL
#ExternalProject_Add_Step(acados export_paths
#        COMMAND echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<INSTALL_DIR>/lib" >> ${CATKIN_DEVEL_PREFIX}/setup.bash
#        COMMAND echo "export ACADOS_SOURCE_DIR=<INSTALL_DIR>" >> ${CATKIN_DEVEL_PREFIX}/setup.bash
#        DEPENDEES pip_install
#)

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build)")
