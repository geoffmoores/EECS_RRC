from typing import Union

from params_proto import Meta # 

from mini_gym.envs.base.legged_robot_config import Cfg


def config_vision60(Cnfg: Union[Cfg, Meta]):
    _ = Cnfg.init_state

    _.pos = [0.0, 0.0, 0.7]  # x,y,z [m]
    _.default_joint_angles = {  # = target angles [rad] when action = 0.0
        # assumption: no need to include fixed joints

        # 0 2 4 6 Hip joints (Motor to upper legs) tried .9, 1
        'joint_0': .9,  # [rad]
        'joint_2': .9,  # [rad]
        'joint_4': .9,  # [rad]
        'joint_6': .9,  # [rad]

        # 1 3 5 7 Knee joints (upper to lower) tried 1.9, 3
        'joint_1': 1.8,  # [rad]
        'joint_3': 1.8,  # [rad]
        'joint_5': 1.8,  # [rad]
        'joint_7': 1.8,  # [rad]

        # Abduction joints (body to hip)
        'joint_8': .17,  # [rad]
        'joint_9': .17,  # [rad]
        'joint_10': -.17,  # [rad]
        'joint_11': -.17,  # [rad]

    }

    _ = Cnfg.control
    _.control_type = 'P'
    _.stiffness = {'joint': 80.}  # [N*m/rad]  ??? Sure
    _.damping = {'joint': .5}  # [N*m*s/rad]  ??? Sure .5 -> 1.0
    # action scale: target angle = actionScale * action + defaultAngle
    _.action_scale = .25 # was 0.25
    _.hip_scale_reduction = 0.5
    # decimation: Number of control action updates @ sim DT per policy DT
    _.decimation = 4

    _ = Cnfg.asset
    _.file = '{MINI_GYM_ROOT_DIR}/resources/robots/vision60/urdf/vision60_v5.urdf'
    _.collapse_fixed_joints = False  ## Added this to support (hopefully) toes...
    _.foot_name = "toe" # calf -> toe
    _.penalize_contacts_on = ["lower","upper"]
    _.terminate_after_contacts_on = ["body"] # base -> body, thigh -> hip
    _.self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
    _.flip_visual_attachments = False
    _.fix_base_link = False

    _ = Cnfg.rewards
    _.soft_dof_pos_limit = 0.9 # % of urdf limits before penalty
    _.base_height_target = 0  ## changed from .5 to 1 (Default)

    _ = Cnfg.rewards.scales
    _.torques = -0.0002
    _.dof_pos_limits = -10.0
    _.orientation = -0. # back to default form -5
    _.base_height = -0. # back to default from -30

    _ = Cnfg.terrain   # investigate for adding obstacles
    _.mesh_type = 'trimesh'
    _.measure_heights = False
    _.terrain_noise_magnitude = 0.0
    _.teleport_robots = True
    _.border_size = 0 # default to 0 from 50

    _.terrain_proportions = [0, 0, 0, 0, 0, 0, 0, 0, 1.0] # various terrain types i think
    _.curriculum = False

    _ = Cnfg.env
    _.num_observations = 42
    _.observe_vel = False 
    _.num_envs = 4000

    # Will need to confirm input space for vision60
    _ = Cnfg.commands
    _.lin_vel_x = [-1.0, 1.0]
    _.lin_vel_y = [-1.0, 1.0]

    _ = Cnfg.commands
    _.heading_command = False
    _.resampling_time = 10.0
    _.command_curriculum = True
    _.num_lin_vel_bins = 30
    _.num_ang_vel_bins = 30
    _.lin_vel_x = [-0.6, 0.6]
    _.lin_vel_y = [-0.6, 0.6]
    _.ang_vel_yaw = [-1, 1]

    # DOMAIN MAYBE ALL GOOD FOR NOW
    _ = Cnfg.domain_rand
    _.randomize_base_mass = True
    _.added_mass_range = [-1, 3]
    _.push_robots = False
    _.max_push_vel_xy = 0.5
    _.randomize_friction = True
    _.friction_range = [0.05, 4.5]
    _.randomize_restitution = True
    _.restitution_range = [0.0, 1.0]
    _.restitution = 0.5  # default terrain restitution
    _.randomize_com_displacement = True
    _.com_displacement_range = [-0.1, 0.1]
    _.randomize_motor_strength = True
    _.motor_strength_range = [0.9, 1.1]
    _.randomize_Kp_factor = False
    _.Kp_factor_range = [0.8, 1.3]
    _.randomize_Kd_factor = False
    _.Kd_factor_range = [0.5, 1.5]
    _.rand_interval_s = 6
