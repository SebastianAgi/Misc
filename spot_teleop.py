class keyboard_teleop:

    def __init__(self, config):

        self.lin_vel = float(config.lin_vel)
        self.ang_vel = float(config.ang_vel)
        self.height_up = float(config.height_up)
        self.height_down = float(config.height_down)

        rospy.init_node('keyboard_teleop')
        self.rate = rospy.Rate(60)  # 60hz is standard for Boston Dynamics robots

        # Initialize service proxies

        # ######################################### Locomotion Services #########################################

        rospy.Service("self_right", Trigger, self.handle_self_right)
        rospy.Service("sit", Trigger, self.handle_sit)
        rospy.Service("stand", Trigger, self.handle_stand)
        rospy.Service("power_on", Trigger, self.handle_power_on)
        rospy.Service("power_off", Trigger, self.handle_safe_power_off)
        rospy.Service("estop/hard", Trigger, self.handle_estop_hard)
        rospy.Service("estop/gentle", Trigger, self.handle_estop_soft)
        rospy.Service("estop/release", Trigger, self.handle_estop_disengage)
        rospy.Service("allow_motion", SetBool, self.handle_allow_motion)
        rospy.Service("stair_mode", SetBool, self.handle_stair_mode)
        rospy.Service("locomotion_mode", SetLocomotion, self.handle_locomotion_mode)
        rospy.Service("swing_height", SetSwingHeight, self.handle_swing_height)
        rospy.Service("velocity_limit", SetVelocity, self.handle_vel_limit)
        rospy.Service("obstacle_params", SetObstacleParams, self.handle_obstacle_params)
        rospy.Service("roll_over_right", Trigger, self.handle_roll_over_right)
        rospy.Service("roll_over_left", Trigger, self.handle_roll_over_left)

        # rospy.Service("posed_stand", PosedStand, self.handle_posed_stand)
        # rospy.Service("terrain_params", SetTerrainParams, self.handle_terrain_params)
        # rospy.Service("list_graph", ListGraph, self.handle_list_graph)


        self.self_right_srv_pub = rospy.ServiceProxy("self_right", Trigger, spot_driver.srv.Stand)
        

        # ######################################### Docking Services #########################################

        # rospy.Service("dock", Dock, self.handle_dock)
        # rospy.Service("undock", Trigger, self.handle_undock)
        # rospy.Service("docking_state", GetDockState, self.handle_get_docking_state)


        # ######################################### Arm Services #########################################

        # rospy.Service("arm_stow", Trigger, self.handle_arm_stow)
        # rospy.Service("arm_unstow", Trigger, self.handle_arm_unstow)
        # rospy.Service("gripper_open", Trigger, self.handle_gripper_open)
        # rospy.Service("gripper_close", Trigger, self.handle_gripper_close)
        # rospy.Service("arm_carry", Trigger, self.handle_arm_carry)
        # rospy.Service("gripper_angle_open", GripperAngleMove, self.handle_gripper_angle_open)
        # rospy.Service("arm_joint_move", ArmJointMovement, self.handle_arm_joint_move)
        # rospy.Service("force_trajectory", ArmForceTrajectory, self.handle_force_trajectory)
        # rospy.Service("gripper_pose", HandPose, self.handle_hand_pose)
        # rospy.Service("grasp_3d", Grasp3d, self.handle_grasp_3d)


