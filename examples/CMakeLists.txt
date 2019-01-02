add_executable(tello_video_test tello_video_test.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test RUNTIME DESTINATION bin)



add_executable(tello_connect_test tello_connect_test.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_connect_test ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_connect_test ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_connect_test RUNTIME DESTINATION bin)



add_executable(tello_drone_mock tello_drone_mock.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_drone_mock ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_drone_mock ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_drone_mock RUNTIME DESTINATION bin)



