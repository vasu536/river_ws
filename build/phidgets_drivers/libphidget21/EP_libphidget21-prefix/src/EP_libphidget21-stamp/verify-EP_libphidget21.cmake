set(file "/home/vasu536/catkin_ws/build/phidgets_drivers/libphidget21/EP_libphidget21-prefix/src/libphidget_2.1.8.20151217.tar.gz")
message(STATUS "verifying file...
     file='${file}'")
set(expect_value "818ab2ff1de92ed9568a206e0e89657f")
set(attempt 0)
set(succeeded 0)
while(${attempt} LESS 3 OR ${attempt} EQUAL 3 AND NOT ${succeeded})
  file(MD5 "${file}" actual_value)
  if("${actual_value}" STREQUAL "${expect_value}")
    set(succeeded 1)
  elseif(${attempt} LESS 3)
    message(STATUS "MD5 hash of ${file}
does not match expected value
  expected: ${expect_value}
    actual: ${actual_value}
Retrying download.
")
    file(REMOVE "${file}")
    execute_process(COMMAND ${CMAKE_COMMAND} -P "/home/vasu536/catkin_ws/build/phidgets_drivers/libphidget21/EP_libphidget21-prefix/src/EP_libphidget21-stamp/download-EP_libphidget21.cmake")
  endif()
  math(EXPR attempt "${attempt} + 1")
endwhile()

if(${succeeded})
  message(STATUS "verifying file... done")
else()
  message(FATAL_ERROR "error: MD5 hash of
  ${file}
does not match expected value
  expected: ${expect_value}
    actual: ${actual_value}
")
endif()
