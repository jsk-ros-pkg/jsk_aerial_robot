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
  LOG_INSTALL_ON
  )

install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build)")
