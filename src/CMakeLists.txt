add_library(gt1920c gt1920c.cc)
target_link_libraries(gt1920c ${CAMERA_LIB} pthread ${OpenCV_LIBS})

#add_executable(test test.cc)
#target_link_libraries(test ${CAMERA_LIB} pthread  ${OpenCV_LIBS})

add_executable(new_driver_test new_driver_test.cc)
target_link_libraries(new_driver_test gt1920c)
