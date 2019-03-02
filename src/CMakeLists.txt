add_library( stereo_vo SHARED
    frame.cpp
    odometry.cpp
    camera.cpp
        map.cpp
        coarse_track_g2o.cpp
    window_ba.cpp
    direct_pose_estimate.cpp
    map_point.cpp
    depth_filter.cpp
)

target_link_libraries( stereo_vo
    ${THIRD_PARTY_LIBS}
)