import pyrealsense2 as rs

# Initialize the RealSense context
context = rs.context()

# Get the first available device
device = context.query_devices()[0]

# Initialize the advanced mode
advanced_mode = rs.rs400_advanced_mode(device)

# Enable advanced mode if not already enabled
if not advanced_mode.is_enabled():
    advanced_mode.toggle_advanced_mode(True)

# Retrieve the current depth table control settings
depth_table_control = advanced_mode.get_depth_table()

# Modify the depthClampMin value
depth_table_control.depthClampMin = 0

# Set the updated depth table control settings back to the camera
advanced_mode.set_depth_table(depth_table_control)

print("depthClampMin has been set to:", depth_table_control.depthClampMin)
