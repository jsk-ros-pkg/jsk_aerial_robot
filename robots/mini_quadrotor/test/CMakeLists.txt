add_rostest(flight_control.test ARGS headless:=true)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -c
    OUTPUT_VARIABLE LSB_RELEASE_CODENAME
)

string(REPLACE "\t" ";" LSB_RELEASE_CODENAME_LIST ${LSB_RELEASE_CODENAME})
list(GET LSB_RELEASE_CODENAME_LIST 1 CODENAME)

# Get the system architecture
execute_process(COMMAND uname -m
    OUTPUT_VARIABLE SYSTEM_ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Skip add_rostest if the system is running Ubuntu Xenial or on an arm64 architecture
if(NOT ${CODENAME} MATCHES "xenial" AND NOT ${SYSTEM_ARCH} MATCHES "aarch64")
  add_rostest(flight_control.test ARGS headless:=true mujoco:=true)
endif()
