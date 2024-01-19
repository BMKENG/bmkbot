
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- 맵 프레임
  tracking_frame = "base_footprint",  -- 추적 프레임
  published_frame = "odom",  -- 게시 프레임
  odom_frame = "odom",  -- 오도메트리 프레임
  provide_odom_frame = false,  -- 오도메트리 프레임 제공
  publish_frame_projected_to_2d = true,  -- 2D로 투영된 프레임 게시
  use_odometry = true,  -- 오도메트리 사용
  use_nav_sat = false,  -- 네비게이션 위성 사용
  use_landmarks = false,  -- 랜드마크 사용
  num_laser_scans = 1,  -- 레이저 스캔 수
  num_multi_echo_laser_scans = 0,  -- 다중 에코 레이저 스캔 수
  num_subdivisions_per_laser_scan = 1,  -- 레이저 스캔당 서브디비전 수
  num_point_clouds = 0,  -- 포인트 클라우드 수
  lookup_transform_timeout_sec = 0.2,  -- 변환 타임아웃 초
  submap_publish_period_sec = 0.3,  -- 서브맵 게시 주기 초
  pose_publish_period_sec = 5e-3,  -- 포즈 게시 주기 초
  trajectory_publish_period_sec = 30e-3,  -- 경로 게시 주기 초
  rangefinder_sampling_ratio = 1.,  -- 레인지파인더 샘플링 비율
  odometry_sampling_ratio = 1.,  -- 오도메트리 샘플링 비율
  fixed_frame_pose_sampling_ratio = 1.,  -- 고정 프레임 포즈 샘플링 비율
  imu_sampling_ratio = 1.,  -- IMU 샘플링 비율
  landmarks_sampling_ratio = 1.,  -- 랜드마크 샘플링 비율
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- 2D 트라젝토리 빌더 사용

TRAJECTORY_BUILDER_2D.min_range = 0.12  -- 최소 범위
TRAJECTORY_BUILDER_2D.max_range = 3.5  -- 최대 범위
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- 누락된 데이터 레이 길이
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- IMU 데이터 사용 안 함
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 온라인 상관 스캔 매칭 사용
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 최대 각도 (라디안)

POSE_GRAPH.constraint_builder.min_score = 0.65  -- 최소 스코어
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- 글로벌 로컬라이제이션 최소 스코어

-- POSE_GRAPH.optimize_every_n_nodes = 0  -- 노드 최적화 간격 설정

return options