# MediaPipe Pose Detection Message
# Contains all 33 pose landmarks as defined by MediaPipe

std_msgs/Header header

# Array of 33 pose landmarks
# Indices follow MediaPipe Pose landmark model:
# 0-10: Face landmarks
# 11-22: Upper body landmarks
# 23-28: Lower body landmarks
# 29-32: Hand landmarks
PoseLandmark[] landmarks

# Overall detection confidence [0.0, 1.0]
float32 confidence

# Whether a pose was detected in the frame
bool detected
