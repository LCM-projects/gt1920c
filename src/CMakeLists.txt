add_executable(test test.cc)
target_link_libraries(test ${CAMERA_LIB} pthread  ${OpenCV_LIBS})
