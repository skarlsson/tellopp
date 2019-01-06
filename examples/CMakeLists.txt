add_executable(tello_video_test_1a tello_video_test_1a.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_1a ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_1a ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_1a RUNTIME DESTINATION bin)



add_executable(tello_video_test_1c tello_video_test_1c.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_1c gainput ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_1c gainputstatic GLU GL Xext X11 ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_1c RUNTIME DESTINATION bin)



add_executable(tello_video_test_1d tello_video_test_1d.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_1d gainput ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_1d gainputstatic GLU GL Xext X11 ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_1d RUNTIME DESTINATION bin)


add_executable(tello_video_test_2 tello_video_test_2.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_video_test_2 ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_video_test_2 ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_video_test_2 RUNTIME DESTINATION bin)


add_executable(tello_connect_test_1a tello_connect_test_1a.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_connect_test_1a ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_connect_test_1a ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_connect_test_1a RUNTIME DESTINATION bin)



add_executable(tello_connect_test_1b tello_connect_test_1b.cpp)

if (LINK_SHARED)
    target_link_libraries(tello_connect_test_1b ${CSI_LIBS_SHARED})
else ()
    target_link_libraries(tello_connect_test_1b ${CSI_LIBS_STATIC})
endif ()

INSTALL(TARGETS tello_connect_test_1b RUNTIME DESTINATION bin)



