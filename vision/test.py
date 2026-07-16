import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)  # Left IR
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)  # Right IR
profile = pipeline.start(config)

color = profile.get_stream(rs.stream.color)
left_ir = profile.get_stream(rs.stream.infrared, 1)
right_ir = profile.get_stream(rs.stream.infrared, 2)

c2l = color.get_extrinsics_to(left_ir).translation
c2r = color.get_extrinsics_to(right_ir).translation

print(f"RGB → Left IR :  X = {c2l[0]*1000:+.1f} mm")
print(f"RGB → Right IR:  X = {c2r[0]*1000:+.1f} mm")

# 모듈 중심 = Left IR과 Right IR의 중간점이라고 가정
mid_x = (c2l[0] + c2r[0]) / 2
print(f"RGB → 모듈 중심 (가정):  X = {mid_x*1000:+.1f} mm")

pipeline.stop()
