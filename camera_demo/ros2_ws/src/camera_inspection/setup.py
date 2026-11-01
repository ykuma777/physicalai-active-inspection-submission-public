from glob import glob
from setuptools import find_packages, setup

setup(
    name="camera_inspection",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/camera_inspection"]),
        ("share/camera_inspection", ["package.xml"]),
        ("share/camera_inspection/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Camera Demo",
    description="Camera observation and image classification with CRANE+ V2 in Gazebo",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "capture_image = camera_inspection.capture_image:main",
        ],
    },
)
