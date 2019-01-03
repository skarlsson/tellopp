add_executable(tello_video_test_1 tello_video_test_1.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_1 ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_1 ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_1 RUNTIME DESTINATION bin)



add_executable(tello_video_test_2 tello_video_test_2.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_2 ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_2 ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_2 RUNTIME DESTINATION bin)


add_executable(tello_connect_test tello_connect_test.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_connect_test ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_connect_test ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_connect_test RUNTIME DESTINATION bin)




