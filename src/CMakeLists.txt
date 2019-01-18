add_library( stereo_vo SHARED
    frame.cpp
    odometry.cpp
    camera.cpp
    local_map.cpp
)

target_link_libraries( stereo_vo
    ${THIRD_PARTY_LIBS}
)