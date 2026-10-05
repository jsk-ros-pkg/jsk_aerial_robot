if(NOT DEFINED URL)
  message(FATAL_ERROR "URL is not defined")
endif()

if(NOT DEFINED OUTPUT)
  message(FATAL_ERROR "OUTPUT is not defined")
endif()

file(DOWNLOAD
  "${URL}"
  "${OUTPUT}"
  SHOW_PROGRESS
  STATUS download_status
)

list(GET download_status 0 status_code)
list(GET download_status 1 status_message)

if(NOT status_code EQUAL 0)
  file(REMOVE "${OUTPUT}")
  message(FATAL_ERROR "Failed to download ${URL}: ${status_message}")
endif()
