from setuptools import find_packages, setup

package_name = "6dof_ros2_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Reza",
    maintainer_email="mrezanvari@gmail.com",
    description="ROS2 Controller for the 6-DOF robotic Arm.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "connect = 6dof_ros2_controller.connect:main",
            "server = 6dof_ros2_controller.server:main",
            "client = 6dof_ros2_controller.client:main",
            "sub.pose = 6dof_ros2_controller.subscriber_pose:main",
            "sub.joints = 6dof_ros2_controller.subscriber_joint_angles:main",
        ],
    },
)
