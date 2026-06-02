idf_component_register(
    SRCS "svc_motion.c" "trajectory.c" "move_queue.c" "cart_path.c"
    INCLUDE_DIRS "include"
    REQUIRES common drv_servo drv_pot dom_kinematics dom_joint svc_safety esp_timer
)
