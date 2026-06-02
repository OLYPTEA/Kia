idf_component_register(
    SRCS
        "test_main.c"
        "test_kinematics.c"
        "test_cobs.c"
        "test_crc32.c"
        "test_ringbuf.c"
        "test_trajectory.c"
    INCLUDE_DIRS "."
    REQUIRES
        unity
        common
        dom_kinematics
        dom_joint
        svc_motion
)
