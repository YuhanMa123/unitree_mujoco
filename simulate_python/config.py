ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml" # G1 29DOF scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True # Virtual spring band for humanoid joint reset

SIMULATE_DT = 0.002  # 500 Hz physics; recompute PD before every step
VIEWER_DT = 0.02  # 50 fps for viewer

# Synchronized observations for example/cpp/build/g1_parkour (loopback UDP).
# No Python ONNX inference is used. Set False for the original DDS-only simulator.
PARKOUR_OBSERVATIONS = True
PARKOUR_OBSERVATION_PORT = 17200
PARKOUR_DEPTH_PREVIEW = True  # Raw depth and the latest encoder frame (OpenCV window)
