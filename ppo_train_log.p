2022-04-06 14:17:56,372 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:17:56,372 checkpoint_interval: 50
2022-04-06 14:17:56,372 clip_param: 0.2
2022-04-06 14:17:56,372 entropy_coef: 0.01
2022-04-06 14:17:56,372 eps: 1e-05
2022-04-06 14:17:56,372 gamma: 0.99
2022-04-06 14:17:56,372 hidden_size: 512
2022-04-06 14:17:56,372 log_file: ppo_train_log.p
2022-04-06 14:17:56,372 log_interval: 1
2022-04-06 14:17:56,372 lr: 0.0007
2022-04-06 14:17:56,372 max_grad_norm: 0.5
2022-04-06 14:17:56,372 num_mini_batch: 10
2022-04-06 14:17:56,372 num_processes: 10
2022-04-06 14:17:56,372 num_steps: 5
2022-04-06 14:17:56,373 num_updates: 10000
2022-04-06 14:17:56,373 opts: []
2022-04-06 14:17:56,373 ppo_epoch: 4
2022-04-06 14:17:56,373 pth_gpu_id: 0
2022-04-06 14:17:56,373 reward_window_size: 50
2022-04-06 14:17:56,373 seed: 100
2022-04-06 14:17:56,373 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:17:56,373 sim_gpu_id: 0
2022-04-06 14:17:56,373 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:17:56,373 tau: 0.95
2022-04-06 14:17:56,373 use_gae: False
2022-04-06 14:17:56,373 use_linear_clip_decay: False
2022-04-06 14:17:56,373 use_linear_lr_decay: False
2022-04-06 14:17:56,373 value_loss_coef: 0.5
2022-04-06 14:19:06,574 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Mobridge', 'Mesic', 'Stilwell', 'Quantico', 'Crandon', 'Annawan', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,581 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Avonia', 'Roeville', 'Adrian', 'Hambleton', 'Spencerville', 'Arkansaw', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,587 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Roxboro', 'Kerrtown', 'Shelbiana', 'Nicut', 'Dryville', 'Micanopy']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,594 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Placida', 'Nemacolin', 'Sodaville', 'Eagerville', 'Bowlus', 'Brevort']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,601 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Maryhill', 'Applewold', 'Ballou', 'Stokes', 'Nimmons', 'Spotswood']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,607 config_env: DATASET:
  CONTENT_SCENES: ['Anaheim', 'Delton', 'Silas', 'Goffs', 'Bolton', 'Nuevo', 'Sanctuary']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,614 config_env: DATASET:
  CONTENT_SCENES: ['Soldier', 'Albertville', 'Woonsocket', 'Convoy', 'Azusa', 'Andover', 'Parole']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,621 config_env: DATASET:
  CONTENT_SCENES: ['Hominy', 'Hillsdale', 'Monson', 'Mifflintown', 'Beach', 'Colebrook', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,628 config_env: DATASET:
  CONTENT_SCENES: ['Oyens', 'Mosinee', 'Angiola', 'Sumas', 'Rosser', 'Hometown', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:19:06,634 config_env: DATASET:
  CONTENT_SCENES: ['Springhill', 'Roane', 'Pettigrew', 'Hainesburg', 'Rancocas', 'Pleasant', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:23:20,058 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:23:20,058 checkpoint_interval: 50
2022-04-06 14:23:20,058 clip_param: 0.2
2022-04-06 14:23:20,059 entropy_coef: 0.01
2022-04-06 14:23:20,059 eps: 1e-05
2022-04-06 14:23:20,059 gamma: 0.99
2022-04-06 14:23:20,059 hidden_size: 512
2022-04-06 14:23:20,059 log_file: ppo_train_log.p
2022-04-06 14:23:20,059 log_interval: 1
2022-04-06 14:23:20,059 lr: 0.0007
2022-04-06 14:23:20,059 max_grad_norm: 0.5
2022-04-06 14:23:20,059 num_mini_batch: 10
2022-04-06 14:23:20,059 num_processes: 10
2022-04-06 14:23:20,059 num_steps: 5
2022-04-06 14:23:20,059 num_updates: 10000
2022-04-06 14:23:20,059 opts: []
2022-04-06 14:23:20,059 ppo_epoch: 4
2022-04-06 14:23:20,059 pth_gpu_id: 0
2022-04-06 14:23:20,059 reward_window_size: 50
2022-04-06 14:23:20,059 seed: 100
2022-04-06 14:23:20,059 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:23:20,059 sim_gpu_id: 0
2022-04-06 14:23:20,059 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:23:20,059 tau: 0.95
2022-04-06 14:23:20,059 use_gae: False
2022-04-06 14:23:20,059 use_linear_clip_decay: False
2022-04-06 14:23:20,059 use_linear_lr_decay: False
2022-04-06 14:23:20,059 value_loss_coef: 0.5
2022-04-06 14:24:30,548 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Mobridge', 'Mesic', 'Stilwell', 'Quantico', 'Crandon', 'Annawan', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,554 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Avonia', 'Roeville', 'Adrian', 'Hambleton', 'Spencerville', 'Arkansaw', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,561 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Roxboro', 'Kerrtown', 'Shelbiana', 'Nicut', 'Dryville', 'Micanopy']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,568 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Placida', 'Nemacolin', 'Sodaville', 'Eagerville', 'Bowlus', 'Brevort']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,574 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Maryhill', 'Applewold', 'Ballou', 'Stokes', 'Nimmons', 'Spotswood']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,581 config_env: DATASET:
  CONTENT_SCENES: ['Anaheim', 'Delton', 'Silas', 'Goffs', 'Bolton', 'Nuevo', 'Sanctuary']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,588 config_env: DATASET:
  CONTENT_SCENES: ['Soldier', 'Albertville', 'Woonsocket', 'Convoy', 'Azusa', 'Andover', 'Parole']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,594 config_env: DATASET:
  CONTENT_SCENES: ['Hominy', 'Hillsdale', 'Monson', 'Mifflintown', 'Beach', 'Colebrook', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,601 config_env: DATASET:
  CONTENT_SCENES: ['Oyens', 'Mosinee', 'Angiola', 'Sumas', 'Rosser', 'Hometown', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:24:30,608 config_env: DATASET:
  CONTENT_SCENES: ['Springhill', 'Roane', 'Pettigrew', 'Hainesburg', 'Rancocas', 'Pleasant', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:26:31,183 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:26:31,183 checkpoint_interval: 50
2022-04-06 14:26:31,183 clip_param: 0.2
2022-04-06 14:26:31,183 entropy_coef: 0.01
2022-04-06 14:26:31,183 eps: 1e-05
2022-04-06 14:26:31,183 gamma: 0.99
2022-04-06 14:26:31,183 hidden_size: 512
2022-04-06 14:26:31,183 log_file: ppo_train_log.p
2022-04-06 14:26:31,183 log_interval: 1
2022-04-06 14:26:31,183 lr: 0.0007
2022-04-06 14:26:31,183 max_grad_norm: 0.5
2022-04-06 14:26:31,183 num_mini_batch: 10
2022-04-06 14:26:31,183 num_processes: 10
2022-04-06 14:26:31,183 num_steps: 5
2022-04-06 14:26:31,183 num_updates: 10000
2022-04-06 14:26:31,183 opts: []
2022-04-06 14:26:31,183 ppo_epoch: 4
2022-04-06 14:26:31,183 pth_gpu_id: 0
2022-04-06 14:26:31,183 reward_window_size: 50
2022-04-06 14:26:31,183 seed: 100
2022-04-06 14:26:31,183 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:26:31,183 sim_gpu_id: 0
2022-04-06 14:26:31,183 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:26:31,183 tau: 0.95
2022-04-06 14:26:31,183 use_gae: False
2022-04-06 14:26:31,183 use_linear_clip_decay: False
2022-04-06 14:26:31,183 use_linear_lr_decay: False
2022-04-06 14:26:31,183 value_loss_coef: 0.5
2022-04-06 14:27:41,249 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Mobridge', 'Mesic', 'Stilwell', 'Quantico', 'Crandon', 'Annawan', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,256 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Avonia', 'Roeville', 'Adrian', 'Hambleton', 'Spencerville', 'Arkansaw', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,263 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Roxboro', 'Kerrtown', 'Shelbiana', 'Nicut', 'Dryville', 'Micanopy']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,270 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Placida', 'Nemacolin', 'Sodaville', 'Eagerville', 'Bowlus', 'Brevort']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,276 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Maryhill', 'Applewold', 'Ballou', 'Stokes', 'Nimmons', 'Spotswood']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,283 config_env: DATASET:
  CONTENT_SCENES: ['Anaheim', 'Delton', 'Silas', 'Goffs', 'Bolton', 'Nuevo', 'Sanctuary']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,290 config_env: DATASET:
  CONTENT_SCENES: ['Soldier', 'Albertville', 'Woonsocket', 'Convoy', 'Azusa', 'Andover', 'Parole']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,296 config_env: DATASET:
  CONTENT_SCENES: ['Hominy', 'Hillsdale', 'Monson', 'Mifflintown', 'Beach', 'Colebrook', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,303 config_env: DATASET:
  CONTENT_SCENES: ['Oyens', 'Mosinee', 'Angiola', 'Sumas', 'Rosser', 'Hometown', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:27:41,310 config_env: DATASET:
  CONTENT_SCENES: ['Springhill', 'Roane', 'Pettigrew', 'Hainesburg', 'Rancocas', 'Pleasant', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,174 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:28:05,174 checkpoint_interval: 50
2022-04-06 14:28:05,174 clip_param: 0.2
2022-04-06 14:28:05,174 entropy_coef: 0.01
2022-04-06 14:28:05,175 eps: 1e-05
2022-04-06 14:28:05,175 gamma: 0.99
2022-04-06 14:28:05,175 hidden_size: 512
2022-04-06 14:28:05,175 log_file: ppo_train_log.p
2022-04-06 14:28:05,175 log_interval: 1
2022-04-06 14:28:05,175 lr: 0.0007
2022-04-06 14:28:05,175 max_grad_norm: 0.5
2022-04-06 14:28:05,175 num_mini_batch: 10
2022-04-06 14:28:05,175 num_processes: 10
2022-04-06 14:28:05,175 num_steps: 5
2022-04-06 14:28:05,175 num_updates: 10000
2022-04-06 14:28:05,175 opts: []
2022-04-06 14:28:05,175 ppo_epoch: 4
2022-04-06 14:28:05,175 pth_gpu_id: 0
2022-04-06 14:28:05,175 reward_window_size: 50
2022-04-06 14:28:05,175 seed: 100
2022-04-06 14:28:05,175 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:28:05,175 sim_gpu_id: 0
2022-04-06 14:28:05,175 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:28:05,175 tau: 0.95
2022-04-06 14:28:05,175 use_gae: False
2022-04-06 14:28:05,175 use_linear_clip_decay: False
2022-04-06 14:28:05,175 use_linear_lr_decay: False
2022-04-06 14:28:05,175 value_loss_coef: 0.5
2022-04-06 14:28:05,204 config_env: DATASET:
  CONTENT_SCENES: ['Sands', 'Swormville']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,211 config_env: DATASET:
  CONTENT_SCENES: ['Denmark', 'Sisters']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,218 config_env: DATASET:
  CONTENT_SCENES: ['Edgemere', 'Mosquito']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,225 config_env: DATASET:
  CONTENT_SCENES: ['Pablo', 'Eastville']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,232 config_env: DATASET:
  CONTENT_SCENES: ['Scioto']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,239 config_env: DATASET:
  CONTENT_SCENES: ['Cantwell']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,246 config_env: DATASET:
  CONTENT_SCENES: ['Elmira']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,252 config_env: DATASET:
  CONTENT_SCENES: ['Ribera']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,259 config_env: DATASET:
  CONTENT_SCENES: ['Eudora']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:05,265 config_env: DATASET:
  CONTENT_SCENES: ['Greigsville']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: val
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:28:13,469 agent number of parameters: 14486661
2022-04-06 14:28:14,173 update: 1	fps: 149.412	
2022-04-06 14:28:14,173 update: 1	env-time: 0.106s	pth-time: 0.501s	frames: 100
2022-04-06 14:28:14,173 Average window size 2 reward: -0.045575
2022-04-06 14:28:14,457 update: 2	fps: 157.367	
2022-04-06 14:28:14,457 update: 2	env-time: 0.158s	pth-time: 0.733s	frames: 150
2022-04-06 14:28:14,457 Average window size 3 reward: -0.056640
2022-04-06 14:28:14,737 update: 3	fps: 162.174	
2022-04-06 14:28:14,737 update: 3	env-time: 0.208s	pth-time: 0.963s	frames: 200
2022-04-06 14:28:14,737 Average window size 4 reward: -0.050189
2022-04-06 14:28:15,010 update: 4	fps: 165.913	
2022-04-06 14:28:15,010 update: 4	env-time: 0.260s	pth-time: 1.185s	frames: 250
2022-04-06 14:28:15,010 Average window size 5 reward: -0.025454
2022-04-06 14:28:15,289 update: 5	fps: 168.043	
2022-04-06 14:28:15,289 update: 5	env-time: 0.311s	pth-time: 1.412s	frames: 300
2022-04-06 14:28:15,289 Average window size 6 reward: -0.014012
2022-04-06 14:28:15,580 update: 6	fps: 168.578	
2022-04-06 14:28:15,580 update: 6	env-time: 0.366s	pth-time: 1.647s	frames: 350
2022-04-06 14:28:15,580 Average window size 7 reward: -0.012335
2022-04-06 14:30:21,801 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:30:21,802 checkpoint_interval: 50
2022-04-06 14:30:21,802 clip_param: 0.2
2022-04-06 14:30:21,802 entropy_coef: 0.01
2022-04-06 14:30:21,802 eps: 1e-05
2022-04-06 14:30:21,802 gamma: 0.99
2022-04-06 14:30:21,802 hidden_size: 512
2022-04-06 14:30:21,802 log_file: ppo_train_log.p
2022-04-06 14:30:21,802 log_interval: 1
2022-04-06 14:30:21,802 lr: 0.0007
2022-04-06 14:30:21,802 max_grad_norm: 0.5
2022-04-06 14:30:21,802 num_mini_batch: 10
2022-04-06 14:30:21,802 num_processes: 10
2022-04-06 14:30:21,802 num_steps: 5
2022-04-06 14:30:21,802 num_updates: 10000
2022-04-06 14:30:21,802 opts: []
2022-04-06 14:30:21,802 ppo_epoch: 4
2022-04-06 14:30:21,802 pth_gpu_id: 0
2022-04-06 14:30:21,802 reward_window_size: 50
2022-04-06 14:30:21,802 seed: 100
2022-04-06 14:30:21,802 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:30:21,802 sim_gpu_id: 0
2022-04-06 14:30:21,802 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:30:21,802 tau: 0.95
2022-04-06 14:30:21,802 use_gae: False
2022-04-06 14:30:21,802 use_linear_clip_decay: False
2022-04-06 14:30:21,802 use_linear_lr_decay: False
2022-04-06 14:30:21,802 value_loss_coef: 0.5
2022-04-06 14:31:31,744 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Mobridge', 'Mesic', 'Stilwell', 'Quantico', 'Crandon', 'Annawan', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,751 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Avonia', 'Roeville', 'Adrian', 'Hambleton', 'Spencerville', 'Arkansaw', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,757 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Roxboro', 'Kerrtown', 'Shelbiana', 'Nicut', 'Dryville', 'Micanopy']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,764 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Placida', 'Nemacolin', 'Sodaville', 'Eagerville', 'Bowlus', 'Brevort']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,771 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Maryhill', 'Applewold', 'Ballou', 'Stokes', 'Nimmons', 'Spotswood']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,778 config_env: DATASET:
  CONTENT_SCENES: ['Anaheim', 'Delton', 'Silas', 'Goffs', 'Bolton', 'Nuevo', 'Sanctuary']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,785 config_env: DATASET:
  CONTENT_SCENES: ['Soldier', 'Albertville', 'Woonsocket', 'Convoy', 'Azusa', 'Andover', 'Parole']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,791 config_env: DATASET:
  CONTENT_SCENES: ['Hominy', 'Hillsdale', 'Monson', 'Mifflintown', 'Beach', 'Colebrook', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,798 config_env: DATASET:
  CONTENT_SCENES: ['Oyens', 'Mosinee', 'Angiola', 'Sumas', 'Rosser', 'Hometown', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:31:31,805 config_env: DATASET:
  CONTENT_SCENES: ['Springhill', 'Roane', 'Pettigrew', 'Hainesburg', 'Rancocas', 'Pleasant', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:38:17,700 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:38:17,717 checkpoint_interval: 50
2022-04-06 14:38:17,717 clip_param: 0.2
2022-04-06 14:38:17,717 entropy_coef: 0.01
2022-04-06 14:38:17,717 eps: 1e-05
2022-04-06 14:38:17,718 gamma: 0.99
2022-04-06 14:38:17,718 hidden_size: 512
2022-04-06 14:38:17,718 log_file: ppo_train_log.p
2022-04-06 14:38:17,718 log_interval: 1
2022-04-06 14:38:17,718 lr: 0.0007
2022-04-06 14:38:17,718 max_grad_norm: 0.5
2022-04-06 14:38:17,718 num_mini_batch: 5
2022-04-06 14:38:17,718 num_processes: 5
2022-04-06 14:38:17,718 num_steps: 5
2022-04-06 14:38:17,718 num_updates: 10000
2022-04-06 14:38:17,719 opts: []
2022-04-06 14:38:17,719 ppo_epoch: 4
2022-04-06 14:38:17,719 pth_gpu_id: 0
2022-04-06 14:38:17,719 reward_window_size: 50
2022-04-06 14:38:17,719 seed: 100
2022-04-06 14:38:17,719 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:38:17,719 sim_gpu_id: 0
2022-04-06 14:38:17,719 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:38:17,719 tau: 0.95
2022-04-06 14:38:17,719 use_gae: False
2022-04-06 14:38:17,720 use_linear_clip_decay: False
2022-04-06 14:38:17,720 use_linear_lr_decay: False
2022-04-06 14:38:17,720 value_loss_coef: 0.5
2022-04-06 14:38:50,731 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:38:50,731 checkpoint_interval: 50
2022-04-06 14:38:50,731 clip_param: 0.2
2022-04-06 14:38:50,731 entropy_coef: 0.01
2022-04-06 14:38:50,731 eps: 1e-05
2022-04-06 14:38:50,731 gamma: 0.99
2022-04-06 14:38:50,731 hidden_size: 512
2022-04-06 14:38:50,731 log_file: ppo_train_log.p
2022-04-06 14:38:50,731 log_interval: 10
2022-04-06 14:38:50,731 lr: 0.0007
2022-04-06 14:38:50,731 max_grad_norm: 0.5
2022-04-06 14:38:50,731 num_mini_batch: 5
2022-04-06 14:38:50,731 num_processes: 5
2022-04-06 14:38:50,731 num_steps: 5
2022-04-06 14:38:50,731 num_updates: 10000
2022-04-06 14:38:50,731 opts: []
2022-04-06 14:38:50,731 ppo_epoch: 4
2022-04-06 14:38:50,731 pth_gpu_id: 0
2022-04-06 14:38:50,731 reward_window_size: 50
2022-04-06 14:38:50,731 seed: 100
2022-04-06 14:38:50,731 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:38:50,731 sim_gpu_id: 0
2022-04-06 14:38:50,731 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:38:50,731 tau: 0.95
2022-04-06 14:38:50,731 use_gae: False
2022-04-06 14:38:50,731 use_linear_clip_decay: False
2022-04-06 14:38:50,731 use_linear_lr_decay: False
2022-04-06 14:38:50,731 value_loss_coef: 0.5
2022-04-06 14:40:01,982 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Anaheim', 'Mobridge', 'Delton', 'Mesic', 'Silas', 'Stilwell', 'Goffs', 'Quantico', 'Bolton', 'Crandon', 'Nuevo', 'Annawan', 'Sanctuary', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:40:01,989 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Soldier', 'Avonia', 'Albertville', 'Roeville', 'Woonsocket', 'Adrian', 'Convoy', 'Hambleton', 'Azusa', 'Spencerville', 'Andover', 'Arkansaw', 'Parole', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:40:01,995 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Hominy', 'Roxboro', 'Hillsdale', 'Kerrtown', 'Monson', 'Shelbiana', 'Mifflintown', 'Nicut', 'Beach', 'Dryville', 'Colebrook', 'Micanopy', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:40:02,002 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Oyens', 'Placida', 'Mosinee', 'Nemacolin', 'Angiola', 'Sodaville', 'Sumas', 'Eagerville', 'Rosser', 'Bowlus', 'Hometown', 'Brevort', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:40:02,009 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Springhill', 'Maryhill', 'Roane', 'Applewold', 'Pettigrew', 'Ballou', 'Hainesburg', 'Stokes', 'Rancocas', 'Nimmons', 'Pleasant', 'Spotswood', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:40:54,262 agent number of parameters: 14486661
2022-04-06 14:40:57,845 update: 10	fps: 79.065	
2022-04-06 14:40:57,845 update: 10	env-time: 0.404s	pth-time: 2.631s	frames: 275
2022-04-06 14:40:57,878 Average window size 11 reward: 0.002358
2022-04-06 14:40:59,521 update: 20	fps: 101.860	
2022-04-06 14:40:59,521 update: 20	env-time: 0.811s	pth-time: 3.866s	frames: 525
2022-04-06 14:40:59,522 Average window size 21 reward: -0.002528
2022-04-06 14:41:01,171 update: 30	fps: 113.896	
2022-04-06 14:41:01,171 update: 30	env-time: 1.165s	pth-time: 5.161s	frames: 775
2022-04-06 14:41:01,171 Average window size 31 reward: -0.007535
2022-04-06 14:41:02,708 update: 40	fps: 122.883	
2022-04-06 14:41:02,708 update: 40	env-time: 1.465s	pth-time: 6.397s	frames: 1025
2022-04-06 14:41:02,708 Average window size 41 reward: -0.005526
2022-04-06 14:41:04,037 update: 50	fps: 131.852	
2022-04-06 14:41:04,037 update: 50	env-time: 1.691s	pth-time: 7.499s	frames: 1275
2022-04-06 14:41:04,037 Average window size 50 reward: -0.007144
2022-04-06 14:41:05,452 update: 60	fps: 137.574	
2022-04-06 14:41:05,452 update: 60	env-time: 1.917s	pth-time: 8.615s	frames: 1525
2022-04-06 14:41:05,452 Average window size 50 reward: -0.006573
2022-04-06 14:41:07,014 update: 70	fps: 140.346	
2022-04-06 14:41:07,014 update: 70	env-time: 2.256s	pth-time: 9.839s	frames: 1775
2022-04-06 14:41:07,014 Average window size 50 reward: -0.064189
2022-04-06 14:41:08,833 update: 80	fps: 139.978	
2022-04-06 14:41:08,833 update: 80	env-time: 2.698s	pth-time: 11.214s	frames: 2025
2022-04-06 14:41:08,833 Average window size 50 reward: -0.040479
2022-04-06 14:41:10,594 update: 90	fps: 140.198	
2022-04-06 14:41:10,594 update: 90	env-time: 3.136s	pth-time: 12.537s	frames: 2275
2022-04-06 14:41:10,594 Average window size 50 reward: -0.028016
2022-04-06 14:41:12,305 update: 100	fps: 140.759	
2022-04-06 14:41:12,305 update: 100	env-time: 3.574s	pth-time: 13.809s	frames: 2525
2022-04-06 14:41:12,305 Average window size 50 reward: -0.023230
2022-04-06 14:41:14,114 update: 110	fps: 140.525	
2022-04-06 14:41:14,114 update: 110	env-time: 4.000s	pth-time: 15.128s	frames: 2775
2022-04-06 14:41:14,114 Average window size 50 reward: -0.020093
2022-04-06 14:41:15,683 update: 120	fps: 141.907	
2022-04-06 14:41:15,684 update: 120	env-time: 4.342s	pth-time: 16.355s	frames: 3025
2022-04-06 14:41:15,684 Average window size 50 reward: -0.005661
2022-04-06 14:41:17,276 update: 130	fps: 142.954	
2022-04-06 14:41:17,276 update: 130	env-time: 4.712s	pth-time: 17.577s	frames: 3275
2022-04-06 14:41:17,276 Average window size 50 reward: -0.010533
2022-04-06 14:41:19,006 update: 140	fps: 143.065	
2022-04-06 14:41:19,006 update: 140	env-time: 5.141s	pth-time: 18.878s	frames: 3525
2022-04-06 14:41:19,006 Average window size 50 reward: -0.009871
2022-04-06 14:41:20,731 update: 150	fps: 143.185	
2022-04-06 14:41:20,731 update: 150	env-time: 5.571s	pth-time: 20.172s	frames: 3775
2022-04-06 14:41:20,732 Average window size 50 reward: -0.008736
2022-04-06 14:41:22,497 update: 160	fps: 143.086	
2022-04-06 14:41:22,497 update: 160	env-time: 6.003s	pth-time: 21.441s	frames: 4025
2022-04-06 14:41:22,497 Average window size 50 reward: -0.005105
2022-04-06 14:41:24,155 update: 170	fps: 143.511	
2022-04-06 14:41:24,155 update: 170	env-time: 6.408s	pth-time: 22.694s	frames: 4275
2022-04-06 14:41:24,155 Average window size 50 reward: -0.007498
2022-04-06 14:41:25,878 update: 180	fps: 143.598	
2022-04-06 14:41:25,878 update: 180	env-time: 6.832s	pth-time: 23.993s	frames: 4525
2022-04-06 14:41:25,879 Average window size 50 reward: -0.003239
2022-04-06 14:41:27,586 update: 190	fps: 143.744	
2022-04-06 14:41:27,586 update: 190	env-time: 7.268s	pth-time: 25.264s	frames: 4775
2022-04-06 14:41:27,586 Average window size 50 reward: -0.003510
2022-04-06 14:41:29,316 update: 200	fps: 143.780	
2022-04-06 14:41:29,316 update: 200	env-time: 7.675s	pth-time: 26.586s	frames: 5025
2022-04-06 14:41:29,316 Average window size 50 reward: -0.003264
2022-04-06 14:41:31,001 update: 210	fps: 143.990	
2022-04-06 14:41:31,001 update: 210	env-time: 8.006s	pth-time: 27.868s	frames: 5275
2022-04-06 14:41:31,001 Average window size 50 reward: -0.010233
2022-04-06 14:41:32,583 update: 220	fps: 144.572	
2022-04-06 14:41:32,583 update: 220	env-time: 8.361s	pth-time: 29.094s	frames: 5525
2022-04-06 14:41:32,583 Average window size 50 reward: -0.016565
2022-04-06 14:41:34,315 update: 230	fps: 144.561	
2022-04-06 14:41:34,315 update: 230	env-time: 8.785s	pth-time: 30.401s	frames: 5775
2022-04-06 14:41:34,315 Average window size 50 reward: -0.020334
2022-04-06 14:41:35,837 update: 240	fps: 145.286	
2022-04-06 14:41:35,837 update: 240	env-time: 9.102s	pth-time: 31.605s	frames: 6025
2022-04-06 14:41:35,837 Average window size 50 reward: -0.029139
2022-04-06 14:41:37,492 update: 250	fps: 145.505	
2022-04-06 14:41:37,492 update: 250	env-time: 9.472s	pth-time: 32.890s	frames: 6275
2022-04-06 14:41:37,492 Average window size 50 reward: -0.033541
2022-04-06 14:41:39,301 update: 260	fps: 145.212	
2022-04-06 14:41:39,301 update: 260	env-time: 9.896s	pth-time: 34.211s	frames: 6525
2022-04-06 14:41:39,301 Average window size 50 reward: -0.025745
2022-04-06 14:41:41,098 update: 270	fps: 144.979	
2022-04-06 14:41:41,098 update: 270	env-time: 10.324s	pth-time: 35.579s	frames: 6775
2022-04-06 14:41:41,098 Average window size 50 reward: -0.017590
2022-04-06 14:41:42,844 update: 280	fps: 144.914	
2022-04-06 14:41:42,844 update: 280	env-time: 10.730s	pth-time: 36.919s	frames: 7025
2022-04-06 14:41:42,844 Average window size 50 reward: -0.016251
2022-04-06 14:41:44,445 update: 290	fps: 145.274	
2022-04-06 14:41:44,445 update: 290	env-time: 11.087s	pth-time: 38.162s	frames: 7275
2022-04-06 14:41:44,445 Average window size 50 reward: -0.017477
2022-04-06 14:41:46,133 update: 300	fps: 145.364	
2022-04-06 14:41:46,133 update: 300	env-time: 11.511s	pth-time: 39.426s	frames: 7525
2022-04-06 14:41:46,133 Average window size 50 reward: -0.015243
2022-04-06 14:41:47,964 update: 310	fps: 145.062	
2022-04-06 14:41:47,965 update: 310	env-time: 11.937s	pth-time: 40.755s	frames: 7775
2022-04-06 14:41:47,965 Average window size 50 reward: -0.012832
2022-04-06 14:41:49,470 update: 320	fps: 145.635	
2022-04-06 14:41:49,471 update: 320	env-time: 12.240s	pth-time: 41.957s	frames: 8025
2022-04-06 14:41:49,471 Average window size 50 reward: -0.016533
2022-04-06 14:41:50,883 update: 330	fps: 146.417	
2022-04-06 14:41:50,883 update: 330	env-time: 12.497s	pth-time: 43.113s	frames: 8275
2022-04-06 14:41:50,884 Average window size 50 reward: -0.019598
2022-04-06 14:41:52,345 update: 340	fps: 147.037	
2022-04-06 14:41:52,346 update: 340	env-time: 12.780s	pth-time: 44.292s	frames: 8525
2022-04-06 14:41:52,346 Average window size 50 reward: -0.036391
2022-04-06 14:41:54,068 update: 350	fps: 146.981	
2022-04-06 14:41:54,069 update: 350	env-time: 13.192s	pth-time: 45.602s	frames: 8775
2022-04-06 14:41:54,069 Average window size 50 reward: -0.034014
2022-04-06 14:41:55,810 update: 360	fps: 146.883	
2022-04-06 14:41:55,810 update: 360	env-time: 13.603s	pth-time: 46.848s	frames: 9025
2022-04-06 14:41:55,810 Average window size 50 reward: -0.032971
2022-04-06 14:41:57,515 update: 370	fps: 146.877	
2022-04-06 14:41:57,515 update: 370	env-time: 14.018s	pth-time: 48.136s	frames: 9275
2022-04-06 14:41:57,515 Average window size 50 reward: -0.024767
2022-04-06 14:41:59,241 update: 380	fps: 146.822	
2022-04-06 14:41:59,241 update: 380	env-time: 14.434s	pth-time: 49.446s	frames: 9525
2022-04-06 14:41:59,242 Average window size 50 reward: -0.019301
2022-04-06 14:42:00,954 update: 390	fps: 146.799	
2022-04-06 14:42:00,954 update: 390	env-time: 14.866s	pth-time: 50.727s	frames: 9775
2022-04-06 14:42:00,954 Average window size 50 reward: -0.006381
2022-04-06 14:42:02,648 update: 400	fps: 146.818	
2022-04-06 14:42:02,649 update: 400	env-time: 15.301s	pth-time: 51.985s	frames: 10025
2022-04-06 14:42:02,649 Average window size 50 reward: -0.009248
2022-04-06 14:42:04,428 update: 410	fps: 146.657	
2022-04-06 14:42:04,428 update: 410	env-time: 15.728s	pth-time: 53.274s	frames: 10275
2022-04-06 14:42:04,428 Average window size 50 reward: -0.009416
2022-04-06 14:42:06,156 update: 420	fps: 146.609	
2022-04-06 14:42:06,156 update: 420	env-time: 16.159s	pth-time: 54.571s	frames: 10525
2022-04-06 14:42:06,156 Average window size 50 reward: -0.012413
2022-04-06 14:42:07,848 update: 430	fps: 146.637	
2022-04-06 14:42:07,848 update: 430	env-time: 16.595s	pth-time: 55.826s	frames: 10775
2022-04-06 14:42:07,848 Average window size 50 reward: -0.012028
2022-04-06 14:42:09,660 update: 440	fps: 146.428	
2022-04-06 14:42:09,660 update: 440	env-time: 17.039s	pth-time: 57.193s	frames: 11025
2022-04-06 14:42:09,660 Average window size 50 reward: -0.012381
2022-04-06 14:42:11,331 update: 450	fps: 146.496	
2022-04-06 14:42:11,332 update: 450	env-time: 17.436s	pth-time: 58.467s	frames: 11275
2022-04-06 14:42:11,332 Average window size 50 reward: -0.008987
2022-04-06 14:42:12,935 update: 460	fps: 146.688	
2022-04-06 14:42:12,935 update: 460	env-time: 17.751s	pth-time: 59.692s	frames: 11525
2022-04-06 14:42:12,936 Average window size 50 reward: -0.008998
2022-04-06 14:42:14,573 update: 470	fps: 146.808	
2022-04-06 14:42:14,573 update: 470	env-time: 18.130s	pth-time: 60.950s	frames: 11775
2022-04-06 14:42:14,574 Average window size 50 reward: -0.011102
2022-04-06 14:42:16,200 update: 480	fps: 146.946	
2022-04-06 14:42:16,200 update: 480	env-time: 18.526s	pth-time: 62.180s	frames: 12025
2022-04-06 14:42:16,200 Average window size 50 reward: -0.012335
2022-04-06 14:42:17,865 update: 490	fps: 147.009	
2022-04-06 14:42:17,865 update: 490	env-time: 18.929s	pth-time: 63.441s	frames: 12275
2022-04-06 14:42:17,865 Average window size 50 reward: -0.014187
2022-04-06 14:42:19,614 update: 500	fps: 146.925	
2022-04-06 14:42:19,614 update: 500	env-time: 19.356s	pth-time: 64.762s	frames: 12525
2022-04-06 14:42:19,614 Average window size 50 reward: -0.014889
2022-04-06 14:42:21,379 update: 510	fps: 146.819	
2022-04-06 14:42:21,379 update: 510	env-time: 19.769s	pth-time: 66.051s	frames: 12775
2022-04-06 14:42:21,379 Average window size 50 reward: -0.007670
2022-04-06 14:42:23,037 update: 520	fps: 146.893	
2022-04-06 14:42:23,037 update: 520	env-time: 20.150s	pth-time: 67.328s	frames: 13025
2022-04-06 14:42:23,037 Average window size 50 reward: -0.007614
2022-04-06 14:42:24,672 update: 530	fps: 147.002	
2022-04-06 14:42:24,672 update: 530	env-time: 20.533s	pth-time: 68.579s	frames: 13275
2022-04-06 14:42:24,672 Average window size 50 reward: -0.010098
2022-04-06 14:42:26,355 update: 540	fps: 147.030	
2022-04-06 14:42:26,355 update: 540	env-time: 20.943s	pth-time: 69.851s	frames: 13525
2022-04-06 14:42:26,355 Average window size 50 reward: -0.010929
2022-04-06 14:42:28,070 update: 550	fps: 147.007	
2022-04-06 14:42:28,070 update: 550	env-time: 21.373s	pth-time: 71.136s	frames: 13775
2022-04-06 14:42:28,070 Average window size 50 reward: -0.009709
2022-04-06 14:42:29,724 update: 560	fps: 147.078	
2022-04-06 14:42:29,724 update: 560	env-time: 21.734s	pth-time: 72.366s	frames: 14025
2022-04-06 14:42:29,724 Average window size 50 reward: -0.018026
2022-04-06 14:42:31,348 update: 570	fps: 147.194	
2022-04-06 14:42:31,348 update: 570	env-time: 22.105s	pth-time: 73.617s	frames: 14275
2022-04-06 14:42:31,348 Average window size 50 reward: -0.015783
2022-04-06 14:43:07,538 checkpoint_folder: ppo_train_ckpt/
2022-04-06 14:43:07,538 checkpoint_interval: 500
2022-04-06 14:43:07,538 clip_param: 0.2
2022-04-06 14:43:07,538 entropy_coef: 0.01
2022-04-06 14:43:07,538 eps: 1e-05
2022-04-06 14:43:07,538 gamma: 0.99
2022-04-06 14:43:07,538 hidden_size: 512
2022-04-06 14:43:07,538 log_file: ppo_train_log.p
2022-04-06 14:43:07,538 log_interval: 10
2022-04-06 14:43:07,538 lr: 0.0007
2022-04-06 14:43:07,538 max_grad_norm: 0.5
2022-04-06 14:43:07,538 num_mini_batch: 5
2022-04-06 14:43:07,538 num_processes: 5
2022-04-06 14:43:07,538 num_steps: 5
2022-04-06 14:43:07,538 num_updates: 10000
2022-04-06 14:43:07,538 opts: []
2022-04-06 14:43:07,539 ppo_epoch: 4
2022-04-06 14:43:07,539 pth_gpu_id: 0
2022-04-06 14:43:07,539 reward_window_size: 50
2022-04-06 14:43:07,539 seed: 100
2022-04-06 14:43:07,539 sensors: RGB_SENSOR,DEPTH_SENSOR
2022-04-06 14:43:07,539 sim_gpu_id: 0
2022-04-06 14:43:07,539 task_config: habitat-lab/configs/tasks/pointnav.yaml
2022-04-06 14:43:07,539 tau: 0.95
2022-04-06 14:43:07,539 use_gae: False
2022-04-06 14:43:07,539 use_linear_clip_decay: False
2022-04-06 14:43:07,539 use_linear_lr_decay: False
2022-04-06 14:43:07,539 value_loss_coef: 0.5
2022-04-06 14:44:17,642 config_env: DATASET:
  CONTENT_SCENES: ['Sawpit', 'Anaheim', 'Mobridge', 'Delton', 'Mesic', 'Silas', 'Stilwell', 'Goffs', 'Quantico', 'Bolton', 'Crandon', 'Nuevo', 'Annawan', 'Sanctuary', 'Seward']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:44:17,649 config_env: DATASET:
  CONTENT_SCENES: ['Capistrano', 'Soldier', 'Avonia', 'Albertville', 'Roeville', 'Woonsocket', 'Adrian', 'Convoy', 'Hambleton', 'Azusa', 'Spencerville', 'Andover', 'Arkansaw', 'Parole', 'Cooperstown']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:44:17,655 config_env: DATASET:
  CONTENT_SCENES: ['Stanleyville', 'Hominy', 'Roxboro', 'Hillsdale', 'Kerrtown', 'Monson', 'Shelbiana', 'Mifflintown', 'Nicut', 'Beach', 'Dryville', 'Colebrook', 'Micanopy', 'Reyno']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:44:17,662 config_env: DATASET:
  CONTENT_SCENES: ['Haxtun', 'Oyens', 'Placida', 'Mosinee', 'Nemacolin', 'Angiola', 'Sodaville', 'Sumas', 'Eagerville', 'Rosser', 'Bowlus', 'Hometown', 'Brevort', 'Dunmor']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:44:17,668 config_env: DATASET:
  CONTENT_SCENES: ['Sasakwa', 'Springhill', 'Maryhill', 'Roane', 'Applewold', 'Pettigrew', 'Ballou', 'Hainesburg', 'Stokes', 'Rancocas', 'Nimmons', 'Pleasant', 'Spotswood', 'Superior']
  DATA_PATH: data/datasets/pointnav/gibson/v1/{split}/{split}.json.gz
  SCENES_DIR: data/scene_datasets
  SPLIT: train
  TYPE: PointNavDataset-v1
ENVIRONMENT:
  ITERATOR_OPTIONS:
    CYCLE: True
    GROUP_BY_SCENE: True
    MAX_SCENE_REPEAT_EPISODES: -1
    MAX_SCENE_REPEAT_STEPS: 10000
    NUM_EPISODE_SAMPLE: -1
    SHUFFLE: True
    STEP_REPETITION_RANGE: 0.2
  MAX_EPISODE_SECONDS: 10000000
  MAX_EPISODE_STEPS: 500
PYROBOT:
  BASE_CONTROLLER: proportional
  BASE_PLANNER: none
  BUMP_SENSOR:
    TYPE: PyRobotBumpSensor
  DEPTH_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    MAX_DEPTH: 5.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    TYPE: PyRobotDepthSensor
    WIDTH: 640
  LOCOBOT:
    ACTIONS: ['BASE_ACTIONS', 'CAMERA_ACTIONS']
    BASE_ACTIONS: ['go_to_relative', 'go_to_absolute']
    CAMERA_ACTIONS: ['set_pan', 'set_tilt', 'set_pan_tilt']
  RGB_SENSOR:
    CENTER_CROP: False
    HEIGHT: 480
    TYPE: PyRobotRGBSensor
    WIDTH: 640
  ROBOT: locobot
  ROBOTS: ['locobot']
  SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR', 'BUMP_SENSOR']
SEED: 100
SIMULATOR:
  ACTION_SPACE_CONFIG: v0
  AGENTS: ['AGENT_0']
  AGENT_0:
    ANGULAR_ACCELERATION: 12.56
    ANGULAR_FRICTION: 1.0
    COEFFICIENT_OF_RESTITUTION: 0.0
    HEIGHT: 1.5
    IS_SET_START_STATE: False
    LINEAR_ACCELERATION: 20.0
    LINEAR_FRICTION: 0.5
    MASS: 32.0
    RADIUS: 0.1
    SENSORS: ['RGB_SENSOR', 'DEPTH_SENSOR']
    START_POSITION: [0, 0, 0]
    START_ROTATION: [0, 0, 0, 1]
  ARM_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_arm_depth
    WIDTH: 640
  ARM_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_arm_rgb
    WIDTH: 640
  DEFAULT_AGENT_ID: 0
  DEPTH_SENSOR:
    HEIGHT: 256
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    WIDTH: 256
  EQUIRECT_DEPTH_SENSOR:
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularDepthSensor
    WIDTH: 640
  EQUIRECT_RGB_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularRGBSensor
    WIDTH: 640
  EQUIRECT_SEMANTIC_SENSOR:
    HEIGHT: 480
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    TYPE: HabitatSimEquirectangularSemanticSensor
    WIDTH: 640
  FISHEYE_DEPTH_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 480
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeDepthSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_RGB_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeRGBSensor
    WIDTH: 640
    XI: -0.27
  FISHEYE_SEMANTIC_SENSOR:
    ALPHA: 0.57
    FOCAL_LENGTH: [364.84, 364.86]
    HEIGHT: 640
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    PRINCIPAL_POINT_OFFSET: None
    SENSOR_MODEL_TYPE: DOUBLE_SPHERE
    TYPE: HabitatSimFisheyeSemanticSensor
    WIDTH: 640
    XI: -0.27
  FORWARD_STEP_SIZE: 0.25
  HABITAT_SIM_V0:
    ALLOW_SLIDING: True
    ENABLE_PHYSICS: False
    GPU_DEVICE_ID: 0
    GPU_GPU: False
    PHYSICS_CONFIG_FILE: ./data/default.physics_config.json
  HEAD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_head_depth
    WIDTH: 640
  HEAD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_head_rgb
    WIDTH: 640
  RGB_SENSOR:
    HEIGHT: 256
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    WIDTH: 256
  SCENE: data/scene_datasets/habitat-test-scenes/van-gogh-room.glb
  SEED: 100
  SEMANTIC_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimSemanticSensor
    WIDTH: 640
  THIRD_DEPTH_SENSOR:
    HEIGHT: 480
    HFOV: 90
    MAX_DEPTH: 10.0
    MIN_DEPTH: 0.0
    NORMALIZE_DEPTH: True
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimDepthSensor
    UUID: robot_third_rgb
    WIDTH: 640
  THIRD_RGB_SENSOR:
    HEIGHT: 480
    HFOV: 90
    ORIENTATION: [0.0, 0.0, 0.0]
    POSITION: [0, 1.25, 0]
    SENSOR_SUBTYPE: PINHOLE
    TYPE: HabitatSimRGBSensor
    UUID: robot_third_rgb
    WIDTH: 640
  TILT_ANGLE: 15
  TURN_ANGLE: 10
  TYPE: Sim-v0
TASK:
  ACTIONS:
    ANSWER:
      TYPE: AnswerAction
    LOOK_DOWN:
      TYPE: LookDownAction
    LOOK_UP:
      TYPE: LookUpAction
    MOVE_FORWARD:
      TYPE: MoveForwardAction
    STOP:
      TYPE: StopAction
    TELEPORT:
      TYPE: TeleportAction
    TURN_LEFT:
      TYPE: TurnLeftAction
    TURN_RIGHT:
      TYPE: TurnRightAction
    VELOCITY_CONTROL:
      ANG_VEL_RANGE: [-10.0, 10.0]
      LIN_VEL_RANGE: [0.0, 0.25]
      MIN_ABS_ANG_SPEED: 1.0
      MIN_ABS_LIN_SPEED: 0.025
      TIME_STEP: 1.0
      TYPE: VelocityAction
  ANSWER_ACCURACY:
    TYPE: AnswerAccuracy
  COLLISIONS:
    TYPE: Collisions
  COMPASS_SENSOR:
    TYPE: CompassSensor
  CORRECT_ANSWER:
    TYPE: CorrectAnswer
  DISTANCE_TO_GOAL:
    DISTANCE_TO: POINT
    TYPE: DistanceToGoal
  EPISODE_INFO:
    TYPE: EpisodeInfo
  GOAL_SENSOR_UUID: pointgoal_with_gps_compass
  GPS_SENSOR:
    DIMENSIONALITY: 2
    TYPE: GPSSensor
  HEADING_SENSOR:
    TYPE: HeadingSensor
  IMAGEGOAL_SENSOR:
    TYPE: ImageGoalSensor
  INSTRUCTION_SENSOR:
    TYPE: InstructionSensor
  INSTRUCTION_SENSOR_UUID: instruction
  MEASUREMENTS: ['DISTANCE_TO_GOAL', 'SUCCESS', 'SPL']
  OBJECTGOAL_SENSOR:
    GOAL_SPEC: TASK_CATEGORY_ID
    GOAL_SPEC_MAX_VAL: 50
    TYPE: ObjectGoalSensor
  POINTGOAL_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalSensor
  POINTGOAL_WITH_GPS_COMPASS_SENSOR:
    DIMENSIONALITY: 2
    GOAL_FORMAT: POLAR
    TYPE: PointGoalWithGPSCompassSensor
  POSSIBLE_ACTIONS: ['STOP', 'MOVE_FORWARD', 'TURN_LEFT', 'TURN_RIGHT']
  PROXIMITY_SENSOR:
    MAX_DETECTION_RADIUS: 2.0
    TYPE: ProximitySensor
  QUESTION_SENSOR:
    TYPE: QuestionSensor
  SENSORS: ['POINTGOAL_WITH_GPS_COMPASS_SENSOR']
  SOFT_SPL:
    TYPE: SoftSPL
  SPL:
    TYPE: SPL
  SUCCESS:
    SUCCESS_DISTANCE: 0.2
    TYPE: Success
  SUCCESS_DISTANCE: 0.2
  TOP_DOWN_MAP:
    DRAW_BORDER: True
    DRAW_GOAL_AABBS: True
    DRAW_GOAL_POSITIONS: True
    DRAW_SHORTEST_PATH: True
    DRAW_SOURCE: True
    DRAW_VIEW_POINTS: True
    FOG_OF_WAR:
      DRAW: True
      FOV: 90
      VISIBILITY_DIST: 5.0
    MAP_PADDING: 3
    MAP_RESOLUTION: 1024
    MAX_EPISODE_STEPS: 1000
    TYPE: TopDownMap
  TYPE: Nav-v0
2022-04-06 14:44:48,166 agent number of parameters: 14486661
2022-04-06 14:44:50,089 update: 10	fps: 144.249	
2022-04-06 14:44:50,089 update: 10	env-time: 0.431s	pth-time: 1.403s	frames: 275
2022-04-06 14:44:50,089 Average window size 11 reward: -0.015994
2022-04-06 14:44:51,803 update: 20	fps: 145.038	
2022-04-06 14:44:51,803 update: 20	env-time: 0.859s	pth-time: 2.687s	frames: 525
2022-04-06 14:44:51,803 Average window size 21 reward: -0.010714
2022-04-06 14:44:53,506 update: 30	fps: 145.575	
2022-04-06 14:44:53,507 update: 30	env-time: 1.292s	pth-time: 3.958s	frames: 775
2022-04-06 14:44:53,507 Average window size 31 reward: -0.009977
2022-04-06 14:44:55,195 update: 40	fps: 146.176	
2022-04-06 14:44:55,195 update: 40	env-time: 1.723s	pth-time: 5.215s	frames: 1025
2022-04-06 14:44:55,195 Average window size 41 reward: -0.007263
2022-04-06 14:44:56,865 update: 50	fps: 146.854	
2022-04-06 14:44:56,865 update: 50	env-time: 2.154s	pth-time: 6.453s	frames: 1275
2022-04-06 14:44:56,865 Average window size 50 reward: -0.005969
2022-04-06 14:44:58,538 update: 60	fps: 147.272	
2022-04-06 14:44:58,627 update: 60	env-time: 2.586s	pth-time: 7.693s	frames: 1525
2022-04-06 14:44:58,627 Average window size 50 reward: -0.006424
2022-04-06 14:45:00,310 update: 70	fps: 146.370	
2022-04-06 14:45:00,310 update: 70	env-time: 3.015s	pth-time: 8.947s	frames: 1775
2022-04-06 14:45:00,310 Average window size 50 reward: -0.007231
2022-04-06 14:45:02,011 update: 80	fps: 146.440	
2022-04-06 14:45:02,011 update: 80	env-time: 3.443s	pth-time: 10.219s	frames: 2025
2022-04-06 14:45:02,011 Average window size 50 reward: -0.005377
2022-04-06 14:45:03,716 update: 90	fps: 146.459	
2022-04-06 14:45:03,716 update: 90	env-time: 3.857s	pth-time: 11.510s	frames: 2275
2022-04-06 14:45:03,717 Average window size 50 reward: -0.006000
2022-04-06 14:45:05,385 update: 100	fps: 146.784	
2022-04-06 14:45:05,385 update: 100	env-time: 4.281s	pth-time: 12.754s	frames: 2525
2022-04-06 14:45:05,385 Average window size 50 reward: -0.006713
2022-04-06 14:45:07,019 update: 110	fps: 147.324	
2022-04-06 14:45:07,019 update: 110	env-time: 4.689s	pth-time: 13.978s	frames: 2775
2022-04-06 14:45:07,019 Average window size 50 reward: -0.005224
2022-04-06 14:45:08,705 update: 120	fps: 147.402	
2022-04-06 14:45:08,705 update: 120	env-time: 5.117s	pth-time: 15.237s	frames: 3025
2022-04-06 14:45:08,705 Average window size 50 reward: -0.005751
2022-04-06 14:45:10,442 update: 130	fps: 147.132	
2022-04-06 14:45:10,442 update: 130	env-time: 5.547s	pth-time: 16.543s	frames: 3275
2022-04-06 14:45:10,442 Average window size 50 reward: -0.004633
2022-04-06 14:45:12,116 update: 140	fps: 147.285	
2022-04-06 14:45:12,116 update: 140	env-time: 5.976s	pth-time: 17.787s	frames: 3525
2022-04-06 14:45:12,116 Average window size 50 reward: -0.005378
2022-04-06 14:45:13,790 update: 150	fps: 147.421	
2022-04-06 14:45:13,790 update: 150	env-time: 6.408s	pth-time: 19.028s	frames: 3775
2022-04-06 14:45:13,790 Average window size 50 reward: -0.004247
2022-04-06 14:45:15,622 update: 160	fps: 146.690	
2022-04-06 14:45:15,622 update: 160	env-time: 6.841s	pth-time: 20.427s	frames: 4025
2022-04-06 14:45:15,622 Average window size 50 reward: -0.007809
2022-04-06 14:45:17,296 update: 170	fps: 146.842	
2022-04-06 14:45:17,296 update: 170	env-time: 7.268s	pth-time: 21.673s	frames: 4275
2022-04-06 14:45:17,296 Average window size 50 reward: -0.008651
2022-04-06 14:45:18,969 update: 180	fps: 146.982	
2022-04-06 14:45:18,969 update: 180	env-time: 7.701s	pth-time: 22.912s	frames: 4525
2022-04-06 14:45:18,969 Average window size 50 reward: -0.011721
2022-04-06 14:45:20,655 update: 190	fps: 147.048	
2022-04-06 14:45:20,655 update: 190	env-time: 8.135s	pth-time: 24.165s	frames: 4775
2022-04-06 14:45:20,655 Average window size 50 reward: -0.013380
2022-04-06 14:45:22,336 update: 200	fps: 147.133	
2022-04-06 14:45:22,336 update: 200	env-time: 8.568s	pth-time: 25.411s	frames: 5025
2022-04-06 14:45:22,336 Average window size 50 reward: -0.012886
2022-04-06 14:45:24,030 update: 210	fps: 147.152	
2022-04-06 14:45:24,030 update: 210	env-time: 8.996s	pth-time: 26.678s	frames: 5275
2022-04-06 14:45:24,030 Average window size 50 reward: -0.009936
2022-04-06 14:45:25,735 update: 220	fps: 147.128	
2022-04-06 14:45:25,735 update: 220	env-time: 9.418s	pth-time: 27.960s	frames: 5525
2022-04-06 14:45:25,736 Average window size 50 reward: -0.010793
2022-04-06 14:45:27,445 update: 230	fps: 147.089	
2022-04-06 14:45:27,445 update: 230	env-time: 9.843s	pth-time: 29.243s	frames: 5775
2022-04-06 14:45:27,445 Average window size 50 reward: -0.009921
2022-04-06 14:45:29,160 update: 240	fps: 147.033	
2022-04-06 14:45:29,160 update: 240	env-time: 10.271s	pth-time: 30.529s	frames: 6025
2022-04-06 14:45:29,160 Average window size 50 reward: -0.010051
2022-04-06 14:45:30,797 update: 250	fps: 147.252	
2022-04-06 14:45:30,797 update: 250	env-time: 10.654s	pth-time: 31.783s	frames: 6275
2022-04-06 14:45:30,797 Average window size 50 reward: -0.012534
2022-04-06 14:45:32,464 update: 260	fps: 147.355	
2022-04-06 14:45:32,464 update: 260	env-time: 11.067s	pth-time: 33.036s	frames: 6525
2022-04-06 14:45:32,464 Average window size 50 reward: -0.013851
2022-04-06 14:45:34,006 update: 270	fps: 147.851	
2022-04-06 14:45:34,006 update: 270	env-time: 11.428s	pth-time: 34.217s	frames: 6775
2022-04-06 14:45:34,006 Average window size 50 reward: -0.013425
2022-04-06 14:45:35,495 update: 280	fps: 148.483	
2022-04-06 14:45:35,495 update: 280	env-time: 11.767s	pth-time: 35.366s	frames: 7025
2022-04-06 14:45:35,495 Average window size 50 reward: -0.017702
2022-04-06 14:45:36,951 update: 290	fps: 149.176	
2022-04-06 14:45:36,951 update: 290	env-time: 12.091s	pth-time: 36.497s	frames: 7275
2022-04-06 14:45:36,951 Average window size 50 reward: -0.020536
2022-04-06 14:45:38,630 update: 300	fps: 149.164	
2022-04-06 14:45:38,631 update: 300	env-time: 12.501s	pth-time: 37.766s	frames: 7525
2022-04-06 14:45:38,631 Average window size 50 reward: -0.020236
2022-04-06 14:45:40,287 update: 310	fps: 149.219	
2022-04-06 14:45:40,288 update: 310	env-time: 12.923s	pth-time: 39.001s	frames: 7775
2022-04-06 14:45:40,288 Average window size 50 reward: -0.022647
2022-04-06 14:45:41,998 update: 320	fps: 149.120	
2022-04-06 14:45:41,998 update: 320	env-time: 13.341s	pth-time: 40.293s	frames: 8025
2022-04-06 14:45:41,999 Average window size 50 reward: -0.022310
2022-04-06 14:45:43,690 update: 330	fps: 149.081	
2022-04-06 14:45:43,690 update: 330	env-time: 13.767s	pth-time: 41.558s	frames: 8275
2022-04-06 14:45:43,690 Average window size 50 reward: -0.018710
2022-04-06 14:45:45,387 update: 340	fps: 149.026	
2022-04-06 14:45:45,388 update: 340	env-time: 14.199s	pth-time: 42.823s	frames: 8525
2022-04-06 14:45:45,388 Average window size 50 reward: -0.014837
2022-04-06 14:45:47,131 update: 350	fps: 148.859	
2022-04-06 14:45:47,131 update: 350	env-time: 14.624s	pth-time: 44.142s	frames: 8775
2022-04-06 14:45:47,132 Average window size 50 reward: -0.013840
2022-04-06 14:45:48,819 update: 360	fps: 148.838	
2022-04-06 14:45:48,819 update: 360	env-time: 15.046s	pth-time: 45.406s	frames: 9025
2022-04-06 14:45:48,820 Average window size 50 reward: -0.012276
2022-04-06 14:45:50,488 update: 370	fps: 148.864	
2022-04-06 14:45:50,488 update: 370	env-time: 15.476s	pth-time: 46.645s	frames: 9275
2022-04-06 14:45:50,488 Average window size 50 reward: -0.009967
2022-04-06 14:45:52,223 update: 380	fps: 148.735	
2022-04-06 14:45:52,223 update: 380	env-time: 15.908s	pth-time: 47.947s	frames: 9525
2022-04-06 14:45:52,223 Average window size 50 reward: -0.007870
2022-04-06 14:45:53,816 update: 390	fps: 148.935	
2022-04-06 14:45:53,816 update: 390	env-time: 16.262s	pth-time: 49.185s	frames: 9775
2022-04-06 14:45:53,816 Average window size 50 reward: -0.006568
2022-04-06 14:45:55,146 update: 400	fps: 149.708	
2022-04-06 14:45:55,147 update: 400	env-time: 16.488s	pth-time: 50.290s	frames: 10025
2022-04-06 14:45:55,147 Average window size 50 reward: -0.010181
2022-04-06 14:45:56,657 update: 410	fps: 150.056	
2022-04-06 14:45:56,658 update: 410	env-time: 16.821s	pth-time: 51.467s	frames: 10275
2022-04-06 14:45:56,658 Average window size 50 reward: -0.015238
2022-04-06 14:45:58,232 update: 420	fps: 150.252	
2022-04-06 14:45:58,232 update: 420	env-time: 17.197s	pth-time: 52.664s	frames: 10525
2022-04-06 14:45:58,232 Average window size 50 reward: -0.016821
2022-04-06 14:45:59,866 update: 430	fps: 150.314	
2022-04-06 14:45:59,866 update: 430	env-time: 17.605s	pth-time: 53.890s	frames: 10775
2022-04-06 14:45:59,866 Average window size 50 reward: -0.015047
2022-04-06 14:46:01,483 update: 440	fps: 150.408	
2022-04-06 14:46:01,484 update: 440	env-time: 18.001s	pth-time: 55.111s	frames: 11025
2022-04-06 14:46:01,484 Average window size 50 reward: -0.021360
2022-04-06 14:46:03,139 update: 450	fps: 150.421	
2022-04-06 14:46:03,139 update: 450	env-time: 18.388s	pth-time: 56.379s	frames: 11275
2022-04-06 14:46:03,139 Average window size 50 reward: -0.015037
2022-04-06 14:46:04,669 update: 460	fps: 150.680	
2022-04-06 14:46:04,670 update: 460	env-time: 18.695s	pth-time: 57.602s	frames: 11525
2022-04-06 14:46:04,670 Average window size 50 reward: -0.008491
2022-04-06 14:46:06,211 update: 470	fps: 150.907	
2022-04-06 14:46:06,211 update: 470	env-time: 19.007s	pth-time: 58.831s	frames: 11775
2022-04-06 14:46:06,211 Average window size 50 reward: -0.011939
2022-04-06 14:46:07,811 update: 480	fps: 151.013	
2022-04-06 14:46:07,812 update: 480	env-time: 19.350s	pth-time: 60.088s	frames: 12025
2022-04-06 14:46:07,812 Average window size 50 reward: -0.019598
2022-04-06 14:46:09,580 update: 490	fps: 150.804	
2022-04-06 14:46:09,580 update: 490	env-time: 19.773s	pth-time: 61.433s	frames: 12275
2022-04-06 14:46:09,580 Average window size 50 reward: -0.010989
2022-04-06 14:46:11,267 update: 500	fps: 150.750	
2022-04-06 14:46:11,267 update: 500	env-time: 20.196s	pth-time: 62.697s	frames: 12525
2022-04-06 14:46:11,268 Average window size 50 reward: -0.008223
2022-04-06 14:46:13,062 update: 510	fps: 150.507	
2022-04-06 14:46:13,063 update: 510	env-time: 20.623s	pth-time: 64.000s	frames: 12775
2022-04-06 14:46:13,063 Average window size 50 reward: -0.004777
2022-04-06 14:46:14,798 update: 520	fps: 150.378	
2022-04-06 14:46:14,798 update: 520	env-time: 21.051s	pth-time: 65.306s	frames: 13025
2022-04-06 14:46:14,798 Average window size 50 reward: -0.004962
2022-04-06 14:46:16,533 update: 530	fps: 150.253	
2022-04-06 14:46:16,534 update: 530	env-time: 21.479s	pth-time: 66.614s	frames: 13275
2022-04-06 14:46:16,534 Average window size 50 reward: -0.002521
2022-04-06 14:46:18,369 update: 540	fps: 149.968	
2022-04-06 14:46:18,369 update: 540	env-time: 21.922s	pth-time: 68.005s	frames: 13525
2022-04-06 14:46:18,369 Average window size 50 reward: -0.006043
2022-04-06 14:46:19,957 update: 550	fps: 150.096	
2022-04-06 14:46:19,958 update: 550	env-time: 22.295s	pth-time: 69.221s	frames: 13775
2022-04-06 14:46:19,958 Average window size 50 reward: -0.008035
2022-04-06 14:46:21,464 update: 560	fps: 150.352	
2022-04-06 14:46:21,464 update: 560	env-time: 22.596s	pth-time: 70.424s	frames: 14025
2022-04-06 14:46:21,464 Average window size 50 reward: -0.012227
2022-04-06 14:46:23,153 update: 570	fps: 150.310	
2022-04-06 14:46:23,153 update: 570	env-time: 22.978s	pth-time: 71.732s	frames: 14275
2022-04-06 14:46:23,153 Average window size 50 reward: -0.014324
2022-04-06 14:46:24,680 update: 580	fps: 150.523	
2022-04-06 14:46:24,680 update: 580	env-time: 23.313s	pth-time: 72.923s	frames: 14525
2022-04-06 14:46:24,680 Average window size 50 reward: -0.013559
2022-04-06 14:46:26,337 update: 590	fps: 150.528	
2022-04-06 14:46:26,337 update: 590	env-time: 23.689s	pth-time: 74.203s	frames: 14775
2022-04-06 14:46:26,337 Average window size 50 reward: -0.020111
2022-04-06 14:46:28,220 update: 600	fps: 150.194	
2022-04-06 14:46:28,220 update: 600	env-time: 24.144s	pth-time: 75.631s	frames: 15025
2022-04-06 14:46:28,221 Average window size 50 reward: -0.018782
2022-04-06 14:46:29,938 update: 610	fps: 150.115	
2022-04-06 14:46:29,938 update: 610	env-time: 24.554s	pth-time: 76.938s	frames: 15275
2022-04-06 14:46:29,938 Average window size 50 reward: -0.017017
2022-04-06 14:46:31,636 update: 620	fps: 150.068	
2022-04-06 14:46:31,636 update: 620	env-time: 24.973s	pth-time: 78.217s	frames: 15525
2022-04-06 14:46:31,636 Average window size 50 reward: -0.015770
2022-04-06 14:46:33,367 update: 630	fps: 149.975	
2022-04-06 14:46:33,367 update: 630	env-time: 25.400s	pth-time: 79.519s	frames: 15775
2022-04-06 14:46:33,367 Average window size 50 reward: -0.015639
2022-04-06 14:46:35,094 update: 640	fps: 149.891	
2022-04-06 14:46:35,094 update: 640	env-time: 25.829s	pth-time: 80.818s	frames: 16025
2022-04-06 14:46:35,094 Average window size 50 reward: -0.010670
2022-04-06 14:46:36,734 update: 650	fps: 149.929	
2022-04-06 14:46:36,734 update: 650	env-time: 26.219s	pth-time: 82.067s	frames: 16275
2022-04-06 14:46:36,734 Average window size 50 reward: -0.012034
2022-04-06 14:46:38,455 update: 660	fps: 149.857	
2022-04-06 14:46:38,455 update: 660	env-time: 26.638s	pth-time: 83.368s	frames: 16525
2022-04-06 14:46:38,455 Average window size 50 reward: -0.010897
2022-04-06 14:46:40,153 update: 670	fps: 149.817	
2022-04-06 14:46:40,153 update: 670	env-time: 27.065s	pth-time: 84.639s	frames: 16775
2022-04-06 14:46:40,153 Average window size 50 reward: -0.008884
2022-04-06 14:46:41,874 update: 680	fps: 149.748	
2022-04-06 14:46:41,874 update: 680	env-time: 27.495s	pth-time: 85.928s	frames: 17025
2022-04-06 14:46:41,874 Average window size 50 reward: -0.009577
2022-04-06 14:46:43,616 update: 690	fps: 149.653	
2022-04-06 14:46:43,616 update: 690	env-time: 27.923s	pth-time: 87.242s	frames: 17275
2022-04-06 14:46:43,616 Average window size 50 reward: -0.007731
2022-04-06 14:46:45,348 update: 700	fps: 149.575	
2022-04-06 14:46:45,348 update: 700	env-time: 28.354s	pth-time: 88.542s	frames: 17525
2022-04-06 14:46:45,348 Average window size 50 reward: -0.006678
2022-04-06 14:46:47,096 update: 710	fps: 149.478	
2022-04-06 14:46:47,097 update: 710	env-time: 28.789s	pth-time: 89.855s	frames: 17775
2022-04-06 14:46:47,097 Average window size 50 reward: -0.006578
2022-04-06 14:46:48,897 update: 720	fps: 149.319	
2022-04-06 14:46:48,897 update: 720	env-time: 29.228s	pth-time: 91.217s	frames: 18025
2022-04-06 14:46:48,898 Average window size 50 reward: -0.005852
2022-04-06 14:46:50,703 update: 730	fps: 149.159	
2022-04-06 14:46:50,703 update: 730	env-time: 29.676s	pth-time: 92.573s	frames: 18275
2022-04-06 14:46:50,703 Average window size 50 reward: -0.002885
2022-04-06 14:46:52,332 update: 740	fps: 149.216	
2022-04-06 14:46:52,332 update: 740	env-time: 30.046s	pth-time: 93.832s	frames: 18525
2022-04-06 14:46:52,332 Average window size 50 reward: -0.005499
2022-04-06 14:46:53,883 update: 750	fps: 149.363	
2022-04-06 14:46:53,883 update: 750	env-time: 30.416s	pth-time: 95.013s	frames: 18775
2022-04-06 14:46:53,883 Average window size 50 reward: -0.012634
2022-04-06 14:46:55,489 update: 760	fps: 149.443	
2022-04-06 14:46:55,489 update: 760	env-time: 30.783s	pth-time: 96.251s	frames: 19025
2022-04-06 14:46:55,489 Average window size 50 reward: -0.018415
2022-04-06 14:46:57,083 update: 770	fps: 149.535	
2022-04-06 14:46:57,083 update: 770	env-time: 31.128s	pth-time: 97.499s	frames: 19275
2022-04-06 14:46:57,083 Average window size 50 reward: -0.012423
2022-04-06 14:46:58,485 update: 780	fps: 149.844	
2022-04-06 14:46:58,485 update: 780	env-time: 31.362s	pth-time: 98.666s	frames: 19525
2022-04-06 14:46:58,485 Average window size 50 reward: -0.027239
2022-04-06 14:46:59,999 update: 790	fps: 150.019	
2022-04-06 14:47:00,000 update: 790	env-time: 31.664s	pth-time: 99.878s	frames: 19775
2022-04-06 14:47:00,000 Average window size 50 reward: -0.027649
2022-04-06 14:47:01,401 update: 800	fps: 150.317	
2022-04-06 14:47:01,401 update: 800	env-time: 31.910s	pth-time: 101.033s	frames: 20025
2022-04-06 14:47:01,401 Average window size 50 reward: -0.015083
2022-04-06 14:47:02,756 update: 810	fps: 150.661	
2022-04-06 14:47:02,756 update: 810	env-time: 32.140s	pth-time: 102.159s	frames: 20275
2022-04-06 14:47:02,757 Average window size 50 reward: -0.005257
2022-04-06 14:47:04,152 update: 820	fps: 150.953	
2022-04-06 14:47:04,153 update: 820	env-time: 32.366s	pth-time: 103.328s	frames: 20525
2022-04-06 14:47:04,153 Average window size 50 reward: -0.189398
2022-04-06 14:47:05,565 update: 830	fps: 151.220	
2022-04-06 14:47:05,566 update: 830	env-time: 32.592s	pth-time: 104.514s	frames: 20775
2022-04-06 14:47:05,566 Average window size 50 reward: -0.074425
2022-04-06 14:47:07,065 update: 840	fps: 151.387	
2022-04-06 14:47:07,065 update: 840	env-time: 32.830s	pth-time: 105.775s	frames: 21025
2022-04-06 14:47:07,065 Average window size 50 reward: -0.124288
2022-04-06 14:47:08,718 update: 850	fps: 151.385	
2022-04-06 14:47:08,718 update: 850	env-time: 33.214s	pth-time: 107.043s	frames: 21275
2022-04-06 14:47:08,718 Average window size 50 reward: -0.068177
2022-04-06 14:47:10,491 update: 860	fps: 151.257	
2022-04-06 14:47:10,491 update: 860	env-time: 33.644s	pth-time: 108.385s	frames: 21525
2022-04-06 14:47:10,491 Average window size 50 reward: -0.023584
2022-04-06 14:47:12,237 update: 870	fps: 151.159	
2022-04-06 14:47:12,237 update: 870	env-time: 34.075s	pth-time: 109.700s	frames: 21775
2022-04-06 14:47:12,237 Average window size 50 reward: -0.017541
2022-04-06 14:47:13,840 update: 880	fps: 151.211	
2022-04-06 14:47:13,840 update: 880	env-time: 34.460s	pth-time: 110.918s	frames: 22025
2022-04-06 14:47:13,840 Average window size 50 reward: -0.008546
2022-04-06 14:47:15,310 update: 890	fps: 151.399	
2022-04-06 14:47:15,310 update: 890	env-time: 34.753s	pth-time: 112.094s	frames: 22275
2022-04-06 14:47:15,310 Average window size 50 reward: -0.004477
2022-04-06 14:47:16,646 update: 900	fps: 151.721	
2022-04-06 14:47:16,646 update: 900	env-time: 34.982s	pth-time: 113.200s	frames: 22525
2022-04-06 14:47:16,646 Average window size 50 reward: 0.005329
2022-04-06 14:47:18,004 update: 910	fps: 152.015	
2022-04-06 14:47:18,004 update: 910	env-time: 35.207s	pth-time: 114.332s	frames: 22775
2022-04-06 14:47:18,004 Average window size 50 reward: 0.009074
2022-04-06 14:47:19,353 update: 920	fps: 152.312	
2022-04-06 14:47:19,353 update: 920	env-time: 35.432s	pth-time: 115.456s	frames: 23025
2022-04-06 14:47:19,353 Average window size 50 reward: 0.001946
2022-04-06 14:47:20,676 update: 930	fps: 152.630	
2022-04-06 14:47:20,676 update: 930	env-time: 35.658s	pth-time: 116.552s	frames: 23275
2022-04-06 14:47:20,676 Average window size 50 reward: -0.048009
2022-04-06 14:47:22,193 update: 940	fps: 152.750	
2022-04-06 14:47:22,193 update: 940	env-time: 35.961s	pth-time: 117.766s	frames: 23525
2022-04-06 14:47:22,193 Average window size 50 reward: -0.215790
2022-04-06 14:47:23,833 update: 950	fps: 152.746	
2022-04-06 14:47:23,833 update: 950	env-time: 36.346s	pth-time: 119.020s	frames: 23775
2022-04-06 14:47:23,834 Average window size 50 reward: -0.062905
2022-04-06 14:47:25,425 update: 960	fps: 152.790	
2022-04-06 14:47:25,425 update: 960	env-time: 36.710s	pth-time: 120.247s	frames: 24025
2022-04-06 14:47:25,425 Average window size 50 reward: -0.042594
2022-04-06 14:47:27,073 update: 970	fps: 152.778	
2022-04-06 14:47:27,073 update: 970	env-time: 37.107s	pth-time: 121.498s	frames: 24275
2022-04-06 14:47:27,073 Average window size 50 reward: -0.039473
2022-04-06 14:47:28,793 update: 980	fps: 152.699	
2022-04-06 14:47:28,793 update: 980	env-time: 37.542s	pth-time: 122.782s	frames: 24525
2022-04-06 14:47:28,793 Average window size 50 reward: -0.025606
2022-04-06 14:47:30,456 update: 990	fps: 152.674	
2022-04-06 14:47:30,456 update: 990	env-time: 37.971s	pth-time: 124.016s	frames: 24775
2022-04-06 14:47:30,456 Average window size 50 reward: -0.008917
2022-04-06 14:47:32,030 update: 1000	fps: 152.734	
2022-04-06 14:47:32,030 update: 1000	env-time: 38.355s	pth-time: 125.205s	frames: 25025
2022-04-06 14:47:32,030 Average window size 50 reward: -0.007122
2022-04-06 14:47:33,586 update: 1010	fps: 152.809	
2022-04-06 14:47:33,586 update: 1010	env-time: 38.674s	pth-time: 126.378s	frames: 25275
2022-04-06 14:47:33,586 Average window size 50 reward: -0.006674
2022-04-06 14:47:35,236 update: 1020	fps: 152.796	
2022-04-06 14:47:35,236 update: 1020	env-time: 39.066s	pth-time: 127.635s	frames: 25525
2022-04-06 14:47:35,236 Average window size 50 reward: -0.005048
2022-04-06 14:47:36,948 update: 1030	fps: 152.727	
2022-04-06 14:47:36,948 update: 1030	env-time: 39.468s	pth-time: 128.946s	frames: 25775
2022-04-06 14:47:36,949 Average window size 50 reward: -0.008734
2022-04-06 14:47:38,720 update: 1040	fps: 152.606	
2022-04-06 14:47:38,721 update: 1040	env-time: 39.889s	pth-time: 130.296s	frames: 26025
2022-04-06 14:47:38,721 Average window size 50 reward: -0.010061
2022-04-06 14:47:40,458 update: 1050	fps: 152.518	
2022-04-06 14:47:40,458 update: 1050	env-time: 40.305s	pth-time: 131.617s	frames: 26275
2022-04-06 14:47:40,458 Average window size 50 reward: -0.022944
2022-04-06 14:47:42,154 update: 1060	fps: 152.468	
2022-04-06 14:47:42,154 update: 1060	env-time: 40.730s	pth-time: 132.887s	frames: 26525
2022-04-06 14:47:42,154 Average window size 50 reward: -0.017676
2022-04-06 14:47:43,822 update: 1070	fps: 152.444	
2022-04-06 14:47:43,822 update: 1070	env-time: 41.137s	pth-time: 134.148s	frames: 26775
2022-04-06 14:47:43,822 Average window size 50 reward: -0.014967
2022-04-06 14:47:45,370 update: 1080	fps: 152.522	
2022-04-06 14:47:45,371 update: 1080	env-time: 41.511s	pth-time: 135.321s	frames: 27025
2022-04-06 14:47:45,371 Average window size 50 reward: -0.013971
2022-04-06 14:47:46,954 update: 1090	fps: 152.569	
2022-04-06 14:47:46,954 update: 1090	env-time: 41.872s	pth-time: 136.543s	frames: 27275
2022-04-06 14:47:46,954 Average window size 50 reward: -0.016051
2022-04-06 14:47:48,615 update: 1100	fps: 152.550	
2022-04-06 14:47:48,615 update: 1100	env-time: 42.261s	pth-time: 137.815s	frames: 27525
2022-04-06 14:47:48,615 Average window size 50 reward: -0.013171
2022-04-06 14:47:50,310 update: 1110	fps: 152.504	
2022-04-06 14:47:50,310 update: 1110	env-time: 42.655s	pth-time: 139.115s	frames: 27775
2022-04-06 14:47:50,310 Average window size 50 reward: -0.016381
2022-04-06 14:47:52,074 update: 1120	fps: 152.400	
2022-04-06 14:47:52,074 update: 1120	env-time: 43.070s	pth-time: 140.464s	frames: 28025
2022-04-06 14:47:52,074 Average window size 50 reward: -0.016493
2022-04-06 14:47:53,862 update: 1130	fps: 152.279	
2022-04-06 14:47:53,862 update: 1130	env-time: 43.511s	pth-time: 141.810s	frames: 28275
2022-04-06 14:47:53,862 Average window size 50 reward: -0.008846
2022-04-06 14:47:55,623 update: 1140	fps: 152.181	
2022-04-06 14:47:55,624 update: 1140	env-time: 43.949s	pth-time: 143.133s	frames: 28525
2022-04-06 14:47:55,624 Average window size 50 reward: -0.006248
2022-04-06 14:47:57,351 update: 1150	fps: 152.113	
2022-04-06 14:47:57,351 update: 1150	env-time: 44.378s	pth-time: 144.432s	frames: 28775
2022-04-06 14:47:57,351 Average window size 50 reward: -0.003977
2022-04-06 14:47:59,115 update: 1160	fps: 152.018	
2022-04-06 14:47:59,115 update: 1160	env-time: 44.802s	pth-time: 145.770s	frames: 29025
2022-04-06 14:47:59,115 Average window size 50 reward: -0.003885
2022-04-06 14:48:00,738 update: 1170	fps: 152.035	
2022-04-06 14:48:00,738 update: 1170	env-time: 45.179s	pth-time: 147.016s	frames: 29275
2022-04-06 14:48:00,738 Average window size 50 reward: -0.005224
2022-04-06 14:48:02,344 update: 1180	fps: 152.064	
2022-04-06 14:48:02,344 update: 1180	env-time: 45.580s	pth-time: 148.221s	frames: 29525
2022-04-06 14:48:02,345 Average window size 50 reward: -0.012612
2022-04-06 14:48:04,091 update: 1190	fps: 151.985	
2022-04-06 14:48:04,091 update: 1190	env-time: 46.013s	pth-time: 149.534s	frames: 29775
2022-04-06 14:48:04,091 Average window size 50 reward: -0.011859
2022-04-06 14:48:05,820 update: 1200	fps: 151.920	
2022-04-06 14:48:05,820 update: 1200	env-time: 46.434s	pth-time: 150.841s	frames: 30025
2022-04-06 14:48:05,820 Average window size 50 reward: -0.009105
2022-04-06 14:48:07,500 update: 1210	fps: 151.894	
2022-04-06 14:48:07,500 update: 1210	env-time: 46.862s	pth-time: 152.092s	frames: 30275
2022-04-06 14:48:07,500 Average window size 50 reward: -0.009155
2022-04-06 14:48:09,289 update: 1220	fps: 151.785	
2022-04-06 14:48:09,289 update: 1220	env-time: 47.302s	pth-time: 153.441s	frames: 30525
2022-04-06 14:48:09,290 Average window size 50 reward: -0.007411
2022-04-06 14:48:11,032 update: 1230	fps: 151.714	
2022-04-06 14:48:11,032 update: 1230	env-time: 47.724s	pth-time: 154.760s	frames: 30775
2022-04-06 14:48:11,032 Average window size 50 reward: -0.002209
2022-04-06 14:48:12,682 update: 1240	fps: 151.712	
2022-04-06 14:48:12,683 update: 1240	env-time: 48.141s	pth-time: 155.994s	frames: 31025
2022-04-06 14:48:12,683 Average window size 50 reward: -0.005792
2022-04-06 14:48:14,404 update: 1250	fps: 151.658	
2022-04-06 14:48:14,404 update: 1250	env-time: 48.573s	pth-time: 157.282s	frames: 31275
2022-04-06 14:48:14,405 Average window size 50 reward: -0.009583
2022-04-06 14:48:16,153 update: 1260	fps: 151.584	
2022-04-06 14:48:16,153 update: 1260	env-time: 48.999s	pth-time: 158.605s	frames: 31525
2022-04-06 14:48:16,153 Average window size 50 reward: -0.009019
2022-04-06 14:48:17,905 update: 1270	fps: 151.510	
2022-04-06 14:48:17,905 update: 1270	env-time: 49.427s	pth-time: 159.928s	frames: 31775
2022-04-06 14:48:17,905 Average window size 50 reward: -0.008726
2022-04-06 14:48:19,638 update: 1280	fps: 151.450	
2022-04-06 14:48:19,638 update: 1280	env-time: 49.857s	pth-time: 161.231s	frames: 32025
2022-04-06 14:48:19,639 Average window size 50 reward: -0.007584
2022-04-06 14:48:21,401 update: 1290	fps: 151.371	
2022-04-06 14:48:21,401 update: 1290	env-time: 50.283s	pth-time: 162.566s	frames: 32275
2022-04-06 14:48:21,401 Average window size 50 reward: -0.005396
2022-04-06 14:48:23,134 update: 1300	fps: 151.313	
2022-04-06 14:48:23,134 update: 1300	env-time: 50.704s	pth-time: 163.878s	frames: 32525
2022-04-06 14:48:23,135 Average window size 50 reward: -0.004006
2022-04-06 14:48:24,827 update: 1310	fps: 151.285	
2022-04-06 14:48:24,827 update: 1310	env-time: 51.125s	pth-time: 165.150s	frames: 32775
2022-04-06 14:48:24,828 Average window size 50 reward: -0.004664
2022-04-06 14:48:26,507 update: 1320	fps: 151.266	
2022-04-06 14:48:26,507 update: 1320	env-time: 51.530s	pth-time: 166.424s	frames: 33025
2022-04-06 14:48:26,507 Average window size 50 reward: -0.002315
2022-04-06 14:48:28,250 update: 1330	fps: 151.204	
2022-04-06 14:48:28,250 update: 1330	env-time: 51.960s	pth-time: 167.736s	frames: 33275
2022-04-06 14:48:28,250 Average window size 50 reward: -0.006204
2022-04-06 14:48:29,939 update: 1340	fps: 151.180	
2022-04-06 14:48:29,939 update: 1340	env-time: 52.368s	pth-time: 169.017s	frames: 33525
2022-04-06 14:48:29,939 Average window size 50 reward: -0.006591
2022-04-06 14:48:31,569 update: 1350	fps: 151.195	
2022-04-06 14:48:31,569 update: 1350	env-time: 52.758s	pth-time: 170.255s	frames: 33775
2022-04-06 14:48:31,569 Average window size 50 reward: -0.004619
2022-04-06 14:48:33,179 update: 1360	fps: 151.224	
2022-04-06 14:48:33,180 update: 1360	env-time: 53.145s	pth-time: 171.478s	frames: 34025
2022-04-06 14:48:33,180 Average window size 50 reward: -0.010791
2022-04-06 14:48:34,634 update: 1370	fps: 151.357	
2022-04-06 14:48:34,634 update: 1370	env-time: 53.465s	pth-time: 172.612s	frames: 34275
2022-04-06 14:48:34,634 Average window size 50 reward: -0.015608
2022-04-06 14:48:36,178 update: 1380	fps: 151.428	
2022-04-06 14:48:36,178 update: 1380	env-time: 53.818s	pth-time: 173.803s	frames: 34525
2022-04-06 14:48:36,179 Average window size 50 reward: -0.018780
2022-04-06 14:48:37,871 update: 1390	fps: 151.401	
2022-04-06 14:48:37,872 update: 1390	env-time: 54.210s	pth-time: 175.103s	frames: 34775
2022-04-06 14:48:37,872 Average window size 50 reward: -0.021602
2022-04-06 14:48:39,604 update: 1400	fps: 151.348	
2022-04-06 14:48:39,604 update: 1400	env-time: 54.639s	pth-time: 176.406s	frames: 35025
2022-04-06 14:48:39,604 Average window size 50 reward: -0.021693
2022-04-06 14:48:41,348 update: 1410	fps: 151.287	
2022-04-06 14:48:41,348 update: 1410	env-time: 55.069s	pth-time: 177.720s	frames: 35275
2022-04-06 14:48:41,348 Average window size 50 reward: -0.013875
2022-04-06 14:48:43,058 update: 1420	fps: 151.250	
2022-04-06 14:48:43,058 update: 1420	env-time: 55.494s	pth-time: 179.004s	frames: 35525
2022-04-06 14:48:43,058 Average window size 50 reward: -0.011373
2022-04-06 14:48:44,794 update: 1430	fps: 151.197	
2022-04-06 14:48:44,794 update: 1430	env-time: 55.920s	pth-time: 180.313s	frames: 35775
2022-04-06 14:48:44,794 Average window size 50 reward: -0.010741
2022-04-06 14:48:46,448 update: 1440	fps: 151.197	
2022-04-06 14:48:46,448 update: 1440	env-time: 56.322s	pth-time: 181.565s	frames: 36025
2022-04-06 14:48:46,448 Average window size 50 reward: -0.008544
2022-04-06 14:48:48,138 update: 1450	fps: 151.174	
2022-04-06 14:48:48,139 update: 1450	env-time: 56.742s	pth-time: 182.835s	frames: 36275
2022-04-06 14:48:48,139 Average window size 50 reward: -0.007728
2022-04-06 14:48:49,866 update: 1460	fps: 151.127	
2022-04-06 14:48:49,867 update: 1460	env-time: 57.168s	pth-time: 184.136s	frames: 36525
2022-04-06 14:48:49,867 Average window size 50 reward: -0.012019
2022-04-06 14:48:51,563 update: 1470	fps: 151.101	
2022-04-06 14:48:51,563 update: 1470	env-time: 57.592s	pth-time: 185.408s	frames: 36775
2022-04-06 14:48:51,563 Average window size 50 reward: -0.013826
2022-04-06 14:48:53,265 update: 1480	fps: 151.072	
2022-04-06 14:48:53,265 update: 1480	env-time: 58.020s	pth-time: 186.681s	frames: 37025
2022-04-06 14:48:53,265 Average window size 50 reward: -0.012476
2022-04-06 14:48:55,000 update: 1490	fps: 151.023	
2022-04-06 14:48:55,000 update: 1490	env-time: 58.448s	pth-time: 187.988s	frames: 37275
2022-04-06 14:48:55,000 Average window size 50 reward: -0.014384
2022-04-06 14:48:56,696 update: 1500	fps: 150.998	
2022-04-06 14:48:56,696 update: 1500	env-time: 58.876s	pth-time: 189.256s	frames: 37525
2022-04-06 14:48:56,696 Average window size 50 reward: -0.010843
2022-04-06 14:48:58,489 update: 1510	fps: 150.915	
2022-04-06 14:48:58,490 update: 1510	env-time: 59.309s	pth-time: 190.552s	frames: 37775
2022-04-06 14:48:58,490 Average window size 50 reward: -0.009226
2022-04-06 14:49:00,197 update: 1520	fps: 150.884	
2022-04-06 14:49:00,198 update: 1520	env-time: 59.728s	pth-time: 191.841s	frames: 38025
2022-04-06 14:49:00,198 Average window size 50 reward: -0.006846
2022-04-06 14:49:01,934 update: 1530	fps: 150.837	
2022-04-06 14:49:01,934 update: 1530	env-time: 60.156s	pth-time: 193.148s	frames: 38275
2022-04-06 14:49:01,934 Average window size 50 reward: -0.009053
2022-04-06 14:49:03,693 update: 1540	fps: 150.777	
2022-04-06 14:49:03,693 update: 1540	env-time: 60.589s	pth-time: 194.474s	frames: 38525
2022-04-06 14:49:03,693 Average window size 50 reward: -0.007318
2022-04-06 14:49:05,425 update: 1550	fps: 150.733	
2022-04-06 14:49:05,425 update: 1550	env-time: 61.021s	pth-time: 195.774s	frames: 38775
2022-04-06 14:49:05,425 Average window size 50 reward: -0.007994
2022-04-06 14:49:07,113 update: 1560	fps: 150.716	
2022-04-06 14:49:07,114 update: 1560	env-time: 61.444s	pth-time: 197.039s	frames: 39025
2022-04-06 14:49:07,114 Average window size 50 reward: -0.007806
2022-04-06 14:49:08,886 update: 1570	fps: 150.650	
2022-04-06 14:49:08,886 update: 1570	env-time: 61.874s	pth-time: 198.380s	frames: 39275
2022-04-06 14:49:08,886 Average window size 50 reward: -0.010523
2022-04-06 14:49:10,665 update: 1580	fps: 150.582	
2022-04-06 14:49:10,665 update: 1580	env-time: 62.311s	pth-time: 199.721s	frames: 39525
2022-04-06 14:49:10,665 Average window size 50 reward: -0.008139
2022-04-06 14:49:12,385 update: 1590	fps: 150.547	
2022-04-06 14:49:12,386 update: 1590	env-time: 62.743s	pth-time: 201.010s	frames: 39775
2022-04-06 14:49:12,386 Average window size 50 reward: -0.008390
2022-04-06 14:49:14,102 update: 1600	fps: 150.515	
2022-04-06 14:49:14,102 update: 1600	env-time: 63.172s	pth-time: 202.297s	frames: 40025
2022-04-06 14:49:14,103 Average window size 50 reward: -0.009608
2022-04-06 14:49:15,822 update: 1610	fps: 150.483	
2022-04-06 14:49:15,822 update: 1610	env-time: 63.593s	pth-time: 203.594s	frames: 40275
2022-04-06 14:49:15,822 Average window size 50 reward: -0.007712
2022-04-06 14:49:17,551 update: 1620	fps: 150.444	
2022-04-06 14:49:17,552 update: 1620	env-time: 64.023s	pth-time: 204.893s	frames: 40525
2022-04-06 14:49:17,552 Average window size 50 reward: -0.005692
2022-04-06 14:49:21,745 update: 1630	fps: 149.052	
2022-04-06 14:49:21,745 update: 1630	env-time: 66.919s	pth-time: 206.190s	frames: 40775
2022-04-06 14:49:21,745 Average window size 50 reward: -0.006439
2022-04-06 14:49:23,467 update: 1640	fps: 149.028	
2022-04-06 14:49:23,467 update: 1640	env-time: 67.343s	pth-time: 207.488s	frames: 41025
2022-04-06 14:49:23,467 Average window size 50 reward: -0.006869
2022-04-06 14:49:25,119 update: 1650	fps: 149.042	
2022-04-06 14:49:25,119 update: 1650	env-time: 67.742s	pth-time: 208.740s	frames: 41275
2022-04-06 14:49:25,119 Average window size 50 reward: -0.004219
2022-04-06 14:49:26,628 update: 1660	fps: 149.131	
2022-04-06 14:49:26,629 update: 1660	env-time: 68.090s	pth-time: 209.901s	frames: 41525
2022-04-06 14:49:26,629 Average window size 50 reward: -0.008143
2022-04-06 14:49:28,131 update: 1670	fps: 149.224	
2022-04-06 14:49:28,131 update: 1670	env-time: 68.418s	pth-time: 211.075s	frames: 41775
2022-04-06 14:49:28,131 Average window size 50 reward: -0.011374
2022-04-06 14:49:29,686 update: 1680	fps: 149.288	
2022-04-06 14:49:29,686 update: 1680	env-time: 68.765s	pth-time: 212.282s	frames: 42025
2022-04-06 14:49:29,687 Average window size 50 reward: -0.010397
2022-04-06 14:49:31,149 update: 1690	fps: 149.399	
2022-04-06 14:49:31,149 update: 1690	env-time: 69.064s	pth-time: 213.446s	frames: 42275
2022-04-06 14:49:31,149 Average window size 50 reward: -0.015145
2022-04-06 14:49:32,780 update: 1700	fps: 149.421	
2022-04-06 14:49:32,781 update: 1700	env-time: 69.419s	pth-time: 214.722s	frames: 42525
2022-04-06 14:49:32,781 Average window size 50 reward: -0.019166
2022-04-06 14:49:34,424 update: 1710	fps: 149.437	
2022-04-06 14:49:34,424 update: 1710	env-time: 69.796s	pth-time: 215.987s	frames: 42775
2022-04-06 14:49:34,424 Average window size 50 reward: -0.014598
2022-04-06 14:49:36,015 update: 1720	fps: 149.479	
2022-04-06 14:49:36,015 update: 1720	env-time: 70.157s	pth-time: 217.217s	frames: 43025
2022-04-06 14:49:36,015 Average window size 50 reward: -0.011222
2022-04-06 14:49:37,634 update: 1730	fps: 149.507	
2022-04-06 14:49:37,634 update: 1730	env-time: 70.513s	pth-time: 218.479s	frames: 43275
2022-04-06 14:49:37,634 Average window size 50 reward: -0.019457
2022-04-06 14:49:39,368 update: 1740	fps: 149.475	
2022-04-06 14:49:39,368 update: 1740	env-time: 70.930s	pth-time: 219.796s	frames: 43525
2022-04-06 14:49:39,368 Average window size 50 reward: -0.012639
2022-04-06 14:49:41,118 update: 1750	fps: 149.436	
2022-04-06 14:49:41,118 update: 1750	env-time: 71.350s	pth-time: 221.125s	frames: 43775
2022-04-06 14:49:41,118 Average window size 50 reward: -0.014306
2022-04-06 14:49:42,885 update: 1760	fps: 149.388	
2022-04-06 14:49:42,885 update: 1760	env-time: 71.785s	pth-time: 222.456s	frames: 44025
2022-04-06 14:49:42,885 Average window size 50 reward: -0.010148
2022-04-06 14:49:44,688 update: 1770	fps: 149.323	
2022-04-06 14:49:44,688 update: 1770	env-time: 72.215s	pth-time: 223.828s	frames: 44275
2022-04-06 14:49:44,688 Average window size 50 reward: -0.009093
2022-04-06 14:49:46,368 update: 1780	fps: 149.320	
2022-04-06 14:49:46,368 update: 1780	env-time: 72.631s	pth-time: 225.091s	frames: 44525
2022-04-06 14:49:46,368 Average window size 50 reward: -0.003276
2022-04-06 14:49:48,044 update: 1790	fps: 149.319	
2022-04-06 14:49:48,044 update: 1790	env-time: 73.044s	pth-time: 226.354s	frames: 44775
2022-04-06 14:49:48,044 Average window size 50 reward: -0.002981
2022-04-06 14:49:49,784 update: 1800	fps: 149.287	
2022-04-06 14:49:49,784 update: 1800	env-time: 73.461s	pth-time: 227.677s	frames: 45025
2022-04-06 14:49:49,784 Average window size 50 reward: -0.002899
2022-04-06 14:49:51,480 update: 1810	fps: 149.276	
2022-04-06 14:49:51,480 update: 1810	env-time: 73.845s	pth-time: 228.988s	frames: 45275
2022-04-06 14:49:51,480 Average window size 50 reward: -0.008804
2022-04-06 14:49:53,202 update: 1820	fps: 149.253	
2022-04-06 14:49:53,202 update: 1820	env-time: 74.234s	pth-time: 230.321s	frames: 45525
2022-04-06 14:49:53,202 Average window size 50 reward: -0.011363
2022-04-06 14:49:54,792 update: 1830	fps: 149.294	
2022-04-06 14:49:54,792 update: 1830	env-time: 74.605s	pth-time: 231.539s	frames: 45775
2022-04-06 14:49:54,792 Average window size 50 reward: -0.013925
2022-04-06 14:49:56,525 update: 1840	fps: 149.266	
2022-04-06 14:49:56,525 update: 1840	env-time: 75.006s	pth-time: 232.870s	frames: 46025
2022-04-06 14:49:56,525 Average window size 50 reward: -0.021010
2022-04-06 14:50:00,860 update: 1850	fps: 147.996	
2022-04-06 14:50:00,860 update: 1850	env-time: 78.074s	pth-time: 234.137s	frames: 46275
2022-04-06 14:50:00,860 Average window size 50 reward: -0.024514
2022-04-06 14:50:02,488 update: 1860	fps: 148.025	
2022-04-06 14:50:02,488 update: 1860	env-time: 78.470s	pth-time: 235.368s	frames: 46525
2022-04-06 14:50:02,488 Average window size 50 reward: -0.020615
2022-04-06 14:50:04,105 update: 1870	fps: 148.058	
2022-04-06 14:50:04,105 update: 1870	env-time: 78.854s	pth-time: 236.601s	frames: 46775
2022-04-06 14:50:04,106 Average window size 50 reward: -0.018396
2022-04-06 14:50:05,822 update: 1880	fps: 148.045	
2022-04-06 14:50:05,822 update: 1880	env-time: 79.274s	pth-time: 237.897s	frames: 47025
2022-04-06 14:50:05,823 Average window size 50 reward: -0.015166
2022-04-06 14:50:07,502 update: 1890	fps: 148.049	
2022-04-06 14:50:07,502 update: 1890	env-time: 79.691s	pth-time: 239.160s	frames: 47275
2022-04-06 14:50:07,502 Average window size 50 reward: -0.012014
2022-04-06 14:50:09,195 update: 1900	fps: 148.047	
2022-04-06 14:50:09,195 update: 1900	env-time: 80.112s	pth-time: 240.431s	frames: 47525
2022-04-06 14:50:09,196 Average window size 50 reward: -0.009133
2022-04-06 14:50:10,875 update: 1910	fps: 148.051	
2022-04-06 14:50:10,875 update: 1910	env-time: 80.533s	pth-time: 241.688s	frames: 47775
2022-04-06 14:50:10,875 Average window size 50 reward: -0.009073
2022-04-06 14:50:12,544 update: 1920	fps: 148.060	
2022-04-06 14:50:12,544 update: 1920	env-time: 80.941s	pth-time: 242.949s	frames: 48025
2022-04-06 14:50:12,544 Average window size 50 reward: -0.009382
2022-04-06 14:50:14,086 update: 1930	fps: 148.127	
2022-04-06 14:50:14,086 update: 1930	env-time: 81.283s	pth-time: 244.148s	frames: 48275
2022-04-06 14:50:14,086 Average window size 50 reward: -0.010036
2022-04-06 14:50:15,694 update: 1940	fps: 148.163	
2022-04-06 14:50:15,694 update: 1940	env-time: 81.681s	pth-time: 245.358s	frames: 48525
2022-04-06 14:50:15,694 Average window size 50 reward: -0.013364
2022-04-06 14:50:17,364 update: 1950	fps: 148.171	
2022-04-06 14:50:17,364 update: 1950	env-time: 82.085s	pth-time: 246.623s	frames: 48775
2022-04-06 14:50:17,364 Average window size 50 reward: -0.010375
2022-04-06 14:50:19,028 update: 1960	fps: 148.181	
2022-04-06 14:50:19,028 update: 1960	env-time: 82.503s	pth-time: 247.869s	frames: 49025
2022-04-06 14:50:19,028 Average window size 50 reward: -0.008290
2022-04-06 14:50:20,732 update: 1970	fps: 148.174	
2022-04-06 14:50:20,732 update: 1970	env-time: 82.923s	pth-time: 249.153s	frames: 49275
2022-04-06 14:50:20,732 Average window size 50 reward: -0.009250
2022-04-06 14:50:22,393 update: 1980	fps: 148.185	
2022-04-06 14:50:22,394 update: 1980	env-time: 83.343s	pth-time: 250.393s	frames: 49525
2022-04-06 14:50:22,394 Average window size 50 reward: -0.009917
2022-04-06 14:50:24,049 update: 1990	fps: 148.199	
2022-04-06 14:50:24,049 update: 1990	env-time: 83.757s	pth-time: 251.634s	frames: 49775
2022-04-06 14:50:24,050 Average window size 50 reward: -0.008100
2022-04-06 14:50:25,770 update: 2000	fps: 148.184	
2022-04-06 14:50:25,770 update: 2000	env-time: 84.181s	pth-time: 252.930s	frames: 50025
2022-04-06 14:50:25,770 Average window size 50 reward: -0.009752
2022-04-06 14:50:27,577 update: 2010	fps: 148.131	
2022-04-06 14:50:27,577 update: 2010	env-time: 84.608s	pth-time: 254.240s	frames: 50275
2022-04-06 14:50:27,577 Average window size 50 reward: -0.009980
2022-04-06 14:50:29,344 update: 2020	fps: 148.097	
2022-04-06 14:50:29,344 update: 2020	env-time: 85.032s	pth-time: 255.582s	frames: 50525
2022-04-06 14:50:29,344 Average window size 50 reward: -0.007933
2022-04-06 14:50:31,072 update: 2030	fps: 148.080	
2022-04-06 14:50:31,072 update: 2030	env-time: 85.457s	pth-time: 256.884s	frames: 50775
2022-04-06 14:50:31,072 Average window size 50 reward: -0.007047
2022-04-06 14:50:32,823 update: 2040	fps: 148.053	
2022-04-06 14:50:32,823 update: 2040	env-time: 85.884s	pth-time: 258.206s	frames: 51025
2022-04-06 14:50:32,823 Average window size 50 reward: -0.006606
2022-04-06 14:50:34,558 update: 2050	fps: 148.033	
2022-04-06 14:50:34,558 update: 2050	env-time: 86.312s	pth-time: 259.513s	frames: 51275
2022-04-06 14:50:34,558 Average window size 50 reward: -0.005889
2022-04-06 14:50:36,288 update: 2060	fps: 148.016	
2022-04-06 14:50:36,288 update: 2060	env-time: 86.736s	pth-time: 260.818s	frames: 51525
2022-04-06 14:50:36,288 Average window size 50 reward: -0.010036
2022-04-06 14:50:38,006 update: 2070	fps: 148.003	
2022-04-06 14:50:38,006 update: 2070	env-time: 87.161s	pth-time: 262.111s	frames: 51775
2022-04-06 14:50:38,007 Average window size 50 reward: -0.014353
2022-04-06 14:50:39,829 update: 2080	fps: 147.947	
2022-04-06 14:50:39,829 update: 2080	env-time: 87.604s	pth-time: 263.491s	frames: 52025
2022-04-06 14:50:39,829 Average window size 50 reward: -0.016602
2022-04-06 14:50:41,567 update: 2090	fps: 147.927	
2022-04-06 14:50:41,567 update: 2090	env-time: 88.039s	pth-time: 264.793s	frames: 52275
2022-04-06 14:50:41,567 Average window size 50 reward: -0.018037
2022-04-06 14:50:43,333 update: 2100	fps: 147.895	
2022-04-06 14:50:43,333 update: 2100	env-time: 88.466s	pth-time: 266.132s	frames: 52525
2022-04-06 14:50:43,334 Average window size 50 reward: -0.016655
2022-04-06 14:50:45,044 update: 2110	fps: 147.887	
2022-04-06 14:50:45,044 update: 2110	env-time: 88.840s	pth-time: 267.468s	frames: 52775
2022-04-06 14:50:45,044 Average window size 50 reward: -0.010322
2022-04-06 14:50:46,801 update: 2120	fps: 147.859	
2022-04-06 14:50:46,801 update: 2120	env-time: 89.227s	pth-time: 268.836s	frames: 53025
2022-04-06 14:50:46,801 Average window size 50 reward: -0.010149
2022-04-06 14:50:48,551 update: 2130	fps: 147.835	
2022-04-06 14:50:48,551 update: 2130	env-time: 89.645s	pth-time: 270.168s	frames: 53275
2022-04-06 14:50:48,551 Average window size 50 reward: -0.006583
2022-04-06 14:50:50,446 update: 2140	fps: 147.752	
2022-04-06 14:50:50,446 update: 2140	env-time: 90.091s	pth-time: 271.617s	frames: 53525
2022-04-06 14:50:50,446 Average window size 50 reward: -0.003447
2022-04-06 14:50:52,327 update: 2150	fps: 147.675	
2022-04-06 14:50:52,328 update: 2150	env-time: 90.538s	pth-time: 273.050s	frames: 53775
2022-04-06 14:50:52,328 Average window size 50 reward: -0.006055
2022-04-06 14:50:54,098 update: 2160	fps: 147.644	
2022-04-06 14:50:54,098 update: 2160	env-time: 90.966s	pth-time: 274.392s	frames: 54025
2022-04-06 14:50:54,098 Average window size 50 reward: -0.011103
2022-04-06 14:50:55,927 update: 2170	fps: 147.589	
2022-04-06 14:50:55,927 update: 2170	env-time: 91.403s	pth-time: 275.784s	frames: 54275
2022-04-06 14:50:55,927 Average window size 50 reward: -0.009472
2022-04-06 14:50:57,673 update: 2180	fps: 147.568	
2022-04-06 14:50:57,673 update: 2180	env-time: 91.829s	pth-time: 277.103s	frames: 54525
2022-04-06 14:50:57,674 Average window size 50 reward: -0.006991
2022-04-06 14:50:59,384 update: 2190	fps: 147.561	
2022-04-06 14:50:59,384 update: 2190	env-time: 92.252s	pth-time: 278.391s	frames: 54775
2022-04-06 14:50:59,384 Average window size 50 reward: -0.007046
2022-04-06 14:51:01,180 update: 2200	fps: 147.521	
2022-04-06 14:51:01,180 update: 2200	env-time: 92.686s	pth-time: 279.752s	frames: 55025
2022-04-06 14:51:01,180 Average window size 50 reward: -0.006091
2022-04-06 14:51:02,977 update: 2210	fps: 147.481	
2022-04-06 14:51:02,977 update: 2210	env-time: 93.112s	pth-time: 281.122s	frames: 55275
2022-04-06 14:51:02,977 Average window size 50 reward: -0.005256
2022-04-06 14:51:04,756 update: 2220	fps: 147.448	
2022-04-06 14:51:04,756 update: 2220	env-time: 93.553s	pth-time: 282.460s	frames: 55525
2022-04-06 14:51:04,756 Average window size 50 reward: -0.006404
2022-04-06 14:51:06,602 update: 2230	fps: 147.390	
2022-04-06 14:51:06,602 update: 2230	env-time: 94.001s	pth-time: 283.857s	frames: 55775
2022-04-06 14:51:06,602 Average window size 50 reward: -0.008428
2022-04-06 14:51:08,385 update: 2240	fps: 147.356	
2022-04-06 14:51:08,385 update: 2240	env-time: 94.430s	pth-time: 285.210s	frames: 56025
2022-04-06 14:51:08,385 Average window size 50 reward: -0.008631
2022-04-06 14:51:10,216 update: 2250	fps: 147.304	
2022-04-06 14:51:10,216 update: 2250	env-time: 94.863s	pth-time: 286.607s	frames: 56275
2022-04-06 14:51:10,216 Average window size 50 reward: -0.008365
2022-04-06 14:51:11,915 update: 2260	fps: 147.303	
2022-04-06 14:51:11,915 update: 2260	env-time: 95.285s	pth-time: 287.884s	frames: 56525
2022-04-06 14:51:11,915 Average window size 50 reward: -0.002945
2022-04-06 14:51:13,601 update: 2270	fps: 147.307	
2022-04-06 14:51:13,602 update: 2270	env-time: 95.705s	pth-time: 289.150s	frames: 56775
2022-04-06 14:51:13,602 Average window size 50 reward: -0.002058
2022-04-06 14:51:15,315 update: 2280	fps: 147.301	
2022-04-06 14:51:15,315 update: 2280	env-time: 96.126s	pth-time: 290.442s	frames: 57025
2022-04-06 14:51:15,316 Average window size 50 reward: -0.001961
2022-04-06 14:51:17,031 update: 2290	fps: 147.294	
2022-04-06 14:51:17,031 update: 2290	env-time: 96.550s	pth-time: 291.733s	frames: 57275
2022-04-06 14:51:17,031 Average window size 50 reward: -0.002746
2022-04-06 14:51:26,374 update: 2300	fps: 144.466	
2022-04-06 14:51:26,374 update: 2300	env-time: 104.675s	pth-time: 292.950s	frames: 57525
2022-04-06 14:51:26,374 Average window size 50 reward: -0.006041
2022-04-06 14:51:27,950 update: 2310	fps: 144.522	
2022-04-06 14:51:27,950 update: 2310	env-time: 105.037s	pth-time: 294.164s	frames: 57775
2022-04-06 14:51:27,950 Average window size 50 reward: -0.016181
2022-04-06 14:51:29,717 update: 2320	fps: 144.508	
2022-04-06 14:51:29,717 update: 2320	env-time: 105.461s	pth-time: 295.506s	frames: 58025
2022-04-06 14:51:29,717 Average window size 50 reward: -0.020001
2022-04-06 14:51:31,533 update: 2330	fps: 144.477	
2022-04-06 14:51:31,533 update: 2330	env-time: 105.897s	pth-time: 296.886s	frames: 58275
2022-04-06 14:51:31,533 Average window size 50 reward: -0.022263
2022-04-06 14:51:33,269 update: 2340	fps: 144.475	
2022-04-06 14:51:33,270 update: 2340	env-time: 106.328s	pth-time: 298.190s	frames: 58525
2022-04-06 14:51:33,270 Average window size 50 reward: -0.022538
2022-04-06 14:51:35,082 update: 2350	fps: 144.446	
2022-04-06 14:51:35,082 update: 2350	env-time: 106.764s	pth-time: 299.566s	frames: 58775
2022-04-06 14:51:35,082 Average window size 50 reward: -0.016362
2022-04-06 14:51:36,867 update: 2360	fps: 144.427	
2022-04-06 14:51:36,867 update: 2360	env-time: 107.196s	pth-time: 300.919s	frames: 59025
2022-04-06 14:51:36,867 Average window size 50 reward: -0.011657
2022-04-06 14:51:38,445 update: 2370	fps: 144.481	
2022-04-06 14:51:38,445 update: 2370	env-time: 107.518s	pth-time: 302.174s	frames: 59275
2022-04-06 14:51:38,445 Average window size 50 reward: -0.012643
2022-04-06 14:51:40,093 update: 2380	fps: 144.510	
2022-04-06 14:51:40,093 update: 2380	env-time: 107.892s	pth-time: 303.448s	frames: 59525
2022-04-06 14:51:40,094 Average window size 50 reward: -0.012853
2022-04-06 14:51:41,823 update: 2390	fps: 144.510	
2022-04-06 14:51:41,823 update: 2390	env-time: 108.272s	pth-time: 304.797s	frames: 59775
2022-04-06 14:51:41,823 Average window size 50 reward: -0.013109
2022-04-06 14:51:43,436 update: 2400	fps: 144.550	
2022-04-06 14:51:43,436 update: 2400	env-time: 108.630s	pth-time: 306.051s	frames: 60025
2022-04-06 14:51:43,436 Average window size 50 reward: -0.020449
2022-04-06 14:51:45,095 update: 2410	fps: 144.575	
2022-04-06 14:51:45,095 update: 2410	env-time: 109.001s	pth-time: 307.339s	frames: 60275
2022-04-06 14:51:45,095 Average window size 50 reward: -0.025940
2022-04-06 14:51:46,727 update: 2420	fps: 144.608	
2022-04-06 14:51:46,727 update: 2420	env-time: 109.383s	pth-time: 308.588s	frames: 60525
2022-04-06 14:51:46,727 Average window size 50 reward: -0.029599
2022-04-06 14:51:48,329 update: 2430	fps: 144.652	
2022-04-06 14:51:48,329 update: 2430	env-time: 109.765s	pth-time: 309.807s	frames: 60775
2022-04-06 14:51:48,329 Average window size 50 reward: -0.024262
2022-04-06 14:51:50,045 update: 2440	fps: 144.656	
2022-04-06 14:51:50,045 update: 2440	env-time: 110.174s	pth-time: 311.114s	frames: 61025
2022-04-06 14:51:50,045 Average window size 50 reward: -0.022393
2022-04-06 14:51:51,912 update: 2450	fps: 144.609	
2022-04-06 14:51:51,913 update: 2450	env-time: 110.613s	pth-time: 312.541s	frames: 61275
2022-04-06 14:51:51,913 Average window size 50 reward: -0.018720
2022-04-06 14:51:53,672 update: 2460	fps: 144.598	
2022-04-06 14:51:53,673 update: 2460	env-time: 111.045s	pth-time: 313.868s	frames: 61525
2022-04-06 14:51:53,673 Average window size 50 reward: -0.010290
2022-04-06 14:51:55,431 update: 2470	fps: 144.588	
2022-04-06 14:51:55,431 update: 2470	env-time: 111.482s	pth-time: 315.190s	frames: 61775
2022-04-06 14:51:55,431 Average window size 50 reward: -0.007450
2022-04-06 14:51:57,047 update: 2480	fps: 144.626	
2022-04-06 14:51:57,047 update: 2480	env-time: 111.853s	pth-time: 316.434s	frames: 62025
2022-04-06 14:51:57,047 Average window size 50 reward: -0.010310
2022-04-06 14:51:58,646 update: 2490	fps: 144.670	
2022-04-06 14:51:58,646 update: 2490	env-time: 112.222s	pth-time: 317.664s	frames: 62275
2022-04-06 14:51:58,646 Average window size 50 reward: -0.007461
2022-04-06 14:52:00,102 update: 2500	fps: 144.761	
2022-04-06 14:52:00,102 update: 2500	env-time: 112.500s	pth-time: 318.841s	frames: 62525
2022-04-06 14:52:00,102 Average window size 50 reward: -0.003448
2022-04-06 14:52:01,473 update: 2510	fps: 144.880	
2022-04-06 14:52:01,474 update: 2510	env-time: 112.721s	pth-time: 319.925s	frames: 62775
2022-04-06 14:52:01,474 Average window size 50 reward: -0.001349
2022-04-06 14:52:02,842 update: 2520	fps: 144.999	
2022-04-06 14:52:02,842 update: 2520	env-time: 112.957s	pth-time: 321.058s	frames: 63025
2022-04-06 14:52:02,842 Average window size 50 reward: -0.004972
2022-04-06 14:52:04,259 update: 2530	fps: 145.101	
2022-04-06 14:52:04,259 update: 2530	env-time: 113.211s	pth-time: 322.220s	frames: 63275
2022-04-06 14:52:04,259 Average window size 50 reward: 0.016706
2022-04-06 14:52:05,578 update: 2540	fps: 145.235	
2022-04-06 14:52:05,579 update: 2540	env-time: 113.433s	pth-time: 323.317s	frames: 63525
2022-04-06 14:52:05,579 Average window size 50 reward: -0.021274
2022-04-06 14:52:07,019 update: 2550	fps: 145.328	
2022-04-06 14:52:07,019 update: 2550	env-time: 113.720s	pth-time: 324.469s	frames: 63775
2022-04-06 14:52:07,019 Average window size 50 reward: -0.025068
2022-04-06 14:52:08,632 update: 2560	fps: 145.363	
2022-04-06 14:52:08,632 update: 2560	env-time: 114.077s	pth-time: 325.725s	frames: 64025
2022-04-06 14:52:08,632 Average window size 50 reward: -0.010282
2022-04-06 14:52:10,215 update: 2570	fps: 145.408	
2022-04-06 14:52:10,215 update: 2570	env-time: 114.416s	pth-time: 326.969s	frames: 64275
2022-04-06 14:52:10,215 Average window size 50 reward: -0.008545
2022-04-06 14:52:11,861 update: 2580	fps: 145.432	
2022-04-06 14:52:11,861 update: 2580	env-time: 114.783s	pth-time: 328.247s	frames: 64525
2022-04-06 14:52:11,862 Average window size 50 reward: -0.026190
2022-04-06 14:52:13,445 update: 2590	fps: 145.476	
2022-04-06 14:52:13,445 update: 2590	env-time: 115.121s	pth-time: 329.492s	frames: 64775
2022-04-06 14:52:13,445 Average window size 50 reward: -0.019221
2022-04-06 14:52:15,050 update: 2600	fps: 145.513	
2022-04-06 14:52:15,050 update: 2600	env-time: 115.459s	pth-time: 330.758s	frames: 65025
2022-04-06 14:52:15,050 Average window size 50 reward: -0.018219
2022-04-06 14:52:16,671 update: 2610	fps: 145.545	
2022-04-06 14:52:16,671 update: 2610	env-time: 115.828s	pth-time: 332.009s	frames: 65275
2022-04-06 14:52:16,671 Average window size 50 reward: -0.027362
2022-04-06 14:52:18,280 update: 2620	fps: 145.580	
2022-04-06 14:52:18,280 update: 2620	env-time: 116.192s	pth-time: 333.254s	frames: 65525
2022-04-06 14:52:18,280 Average window size 50 reward: -0.018660
2022-04-06 14:52:19,873 update: 2630	fps: 145.620	
2022-04-06 14:52:19,873 update: 2630	env-time: 116.537s	pth-time: 334.502s	frames: 65775
2022-04-06 14:52:19,873 Average window size 50 reward: -0.016573
2022-04-06 14:52:21,484 update: 2640	fps: 145.654	
2022-04-06 14:52:21,484 update: 2640	env-time: 116.892s	pth-time: 335.758s	frames: 66025
2022-04-06 14:52:21,484 Average window size 50 reward: -0.022715
