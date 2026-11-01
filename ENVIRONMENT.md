# Recorded environments

Robot runtime: Ubuntu22.04, ROS2 Humble, Ignition Fortress, MoveIt2, ros2_control, NumPy1.24.4, OpenCV4.5.4, PyYAML5.4.1. Existing image/package builds were used, not rebuilt in this session.

Offline render host: Windows, Python3.12.10, NumPy2.5.2, OpenCV5.0.0, Pillow12.3.0, Matplotlib (see final provenance). Japanese system font required. No font file is redistributed. FFmpeg4.4.2/libx264 and Poppler were already available in the container for encoding/PDF checks.

The minimal saved-inference verifier needs only Python and NumPy. No training or ROS occurs when running it.
