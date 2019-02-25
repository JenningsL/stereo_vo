add_library( stereo_vo SHARED
    frame.cpp
    odometry.cpp
    camera.cpp
        map.cpp
    g2o_types.cpp
    window_ba.cpp
    direct_pose_estimate.cpp
    map_point.cpp
    depth_filter.cpp
)

target_link_libraries( stereo_vo
    ${THIRD_PARTY_LIBS}
)