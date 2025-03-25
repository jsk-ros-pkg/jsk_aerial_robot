cmake_minimum_required(VERSION 2.8.3)
project(acados)
include(ExternalProject)

ExternalProject_Add(
  acados
  GIT_REPOSITORY https://github.com/acados/acados.git
  GIT_TAG v0.3.3
  CMAKE_ARGS -DACADOS_WITH_QPOASES=ON  -DACADOS_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_INSTALL ON
  )

## generate python interface
#ExternalProject_Add_Step(acados pip_install
#        COMMAND pip install -e <BINARY_DIR>/../acados/interfaces/acados_template
#        DEPENDEES install
#)
#
## TODO: download tera_renderer
## TODO: change lib path. I met this error: OSError: /home/li-jinjie/ros1/jsk_aerial_robot_ws/build/acados/acados-prefix/src/acados/lib/../lib/libacados.so: cannot open shared object file: No such file or directory
#
## add path to acados to .bashrc
#execute_process(
#        COMMAND grep -q "ACADOS_SOURCE_DIR" $ENV{HOME}/.bashrc
#        RESULT_VARIABLE result_bashrc
#)
#if(${result_bashrc})
#    ExternalProject_Add_Step(acados export_paths
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            COMMAND echo "# >>>>> Automatically adding ACADOS_SOURCE_DIR to .bashrc" >> $ENV{HOME}/.bashrc
#            COMMAND echo "export ACADOS_SOURCE_DIR=<INSTALL_DIR>/src/acados" >> $ENV{HOME}/.bashrc
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            DEPENDEES pip_install
#    )
#    message("ACADOS_SOURCE_DIR not found in .bashrc, automatically adding it.")
#endif()
#
## add path to acados to .zshrc
#execute_process(
#        COMMAND grep -q "ACADOS_SOURCE_DIR" $ENV{HOME}/.zshrc
#        RESULT_VARIABLE result_zshrc
#)
#if(${result_zshrc})
#    ExternalProject_Add_Step(acados export_paths_zsh
#            COMMAND echo "" >> $ENV{HOME}/.bashrc
#            COMMAND echo "# >>>>> Automatically adding ACADOS_SOURCE_DIR to .zshrc" >> $ENV{HOME}/.zshrc
#            COMMAND echo "export ACADOS_SOURCE_DIR=<INSTALL_DIR>/src/acados" >> $ENV{HOME}/.zshrc
#            COMMAND echo "" >> $ENV{HOME}/.zshrc
#            DEPENDEES pip_install
#    )
#    message("ACADOS_SOURCE_DIR not found in .zshrc, automatically adding it.")
#endif()


install(CODE "execute_process(COMMAND make install WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build)")
