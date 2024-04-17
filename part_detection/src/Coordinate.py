import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Create a pipeline profile object
profile = pipeline.get_active_profile()

# Get the depth sensor and its intrinsics
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
depth_intrinsics = rs.video_stream_profile(profile.get_stream(rs.stream.depth)).get_intrinsics()


# Function to convert pixel coordinates to world coordinates
def pixel_to_world(pixel_coords, depth_image):
    depth_value = depth_image[pixel_coords[1], pixel_coords[0]] * depth_scale
    world_coords = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [pixel_coords[0], pixel_coords[1]], depth_value)
    return world_coords


try:
    while True:
        # Wait for a new frame
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convert depth frame to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Example pixel coordinates
        pixel_coords = [320, 240]  # Adjust as needed

        # Convert pixel coordinates to world coordinates
        world_coords = pixel_to_world(pixel_coords, depth_image)
        print("Pixel coordinates:", pixel_coords)
        print("World coordinates:", world_coords)

        # Display depth image
        cv2.imshow('Depth Image', depth_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    #cv2.destroyAllWindows()
