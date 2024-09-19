add_rostest(flight_control.test ARGS headless:=true)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -c
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME
)

string(REPLACE "\t" ";" LSB_RELEASE_CODENAME_LIST ${LSB_RELEASE_CODENAME})
list(GET LSB_RELEASE_CODENAME_LIST 1 CODENAME)

if(NOT ${CODENAME} MATCHES "xenial")
  add_rostest(flight_control.test ARGS headless:=true mujoco:=true)
endif()
