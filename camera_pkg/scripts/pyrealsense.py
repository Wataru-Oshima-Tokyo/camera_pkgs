import pyrealsense2 as rs

width = 848
height = 480
fps = 30

config = rs.config()
config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

pipeline = rs.pipeline()
profile = pipeline.start(config)
depth_intrinsics = rs.video_stream_profile(profile.get_stream(rs.stream.depth)).get_intrinsics()
color_intrinsics = rs.video_stream_profile(profile.get_stream(rs.stream.color)).get_intrinsics()

print("depth_intrinsics")
print(depth_intrinsics)
print()
print("color_intrinsics")
print(color_intrinsics)

