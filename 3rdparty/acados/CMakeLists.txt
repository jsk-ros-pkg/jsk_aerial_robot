cmake_minimum_required(VERSION 3.5.1)
project(acados)
include(ExternalProject)

set(ACADOS_EXTERNAL_SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(TERA_RENDERER_ARCH "linux-arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(TERA_RENDERER_ARCH "linux-amd64")
else()
  message(FATAL_ERROR "Unsupported architecture for acados t_renderer: ${CMAKE_SYSTEM_PROCESSOR}. You may need to manually build it.")
endif()

set(TERA_RENDERER_VERSION "0.2.0")
set(TERA_RENDERER_URL
  "https://github.com/acados/tera_renderer/releases/download/v${TERA_RENDERER_VERSION}/t_renderer-v${TERA_RENDERER_VERSION}-${TERA_RENDERER_ARCH}"
)

ExternalProject_Add(
  acados
  GIT_REPOSITORY https://github.com/acados/acados.git
  GIT_TAG v0.5.0
  CMAKE_ARGS -DACADOS_WITH_QPOASES=ON  -DACADOS_INSTALL_DIR=${CMAKE_INSTALL_PREFIX}
  INSTALL_COMMAND echo "install"
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  LOG_INSTALL ON
  )

ExternalProject_Add_Step(acados sync_shared_libraries
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_EXTERNAL_SOURCE_DIR}/lib
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build/acados/libacados.so
    ${ACADOS_EXTERNAL_SOURCE_DIR}/lib/libacados.so
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build/external/blasfeo/libblasfeo.so
    ${ACADOS_EXTERNAL_SOURCE_DIR}/lib/libblasfeo.so
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build/external/hpipm/libhpipm.so
    ${ACADOS_EXTERNAL_SOURCE_DIR}/lib/libhpipm.so
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/acados-prefix/src/acados-build/external/qpoases/lib/libqpOASES_e.so
    ${ACADOS_EXTERNAL_SOURCE_DIR}/lib/libqpOASES_e.so
  DEPENDEES build
  DEPENDERS install
  LOG 1
)

ExternalProject_Add_Step(acados sync_headers
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_EXTERNAL_SOURCE_DIR}/include
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${ACADOS_EXTERNAL_SOURCE_DIR}/acados
    ${ACADOS_EXTERNAL_SOURCE_DIR}/include/acados
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${ACADOS_EXTERNAL_SOURCE_DIR}/interfaces/acados_c
    ${ACADOS_EXTERNAL_SOURCE_DIR}/include/acados_c
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_EXTERNAL_SOURCE_DIR}/include/blasfeo
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${ACADOS_EXTERNAL_SOURCE_DIR}/external/blasfeo/include
    ${ACADOS_EXTERNAL_SOURCE_DIR}/include/blasfeo/include
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_EXTERNAL_SOURCE_DIR}/include/hpipm
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${ACADOS_EXTERNAL_SOURCE_DIR}/external/hpipm/include
    ${ACADOS_EXTERNAL_SOURCE_DIR}/include/hpipm/include
  DEPENDEES build
  DEPENDERS install
  LOG 1
)

ExternalProject_Add_Step(acados download_tera_renderer
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ACADOS_EXTERNAL_SOURCE_DIR}/bin
  COMMAND ${CMAKE_COMMAND}
    -DURL=${TERA_RENDERER_URL}
    -DOUTPUT=${ACADOS_EXTERNAL_SOURCE_DIR}/bin/t_renderer
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/download_file.cmake
  COMMAND chmod +x ${ACADOS_EXTERNAL_SOURCE_DIR}/bin/t_renderer
  DEPENDEES install
  LOG 1
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

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/50.acados.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/50.acados.sh
  @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/50.acados.sh
  DESTINATION etc/catkin/profile.d
)
