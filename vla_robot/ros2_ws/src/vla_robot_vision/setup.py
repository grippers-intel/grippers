from setuptools import find_packages, setup

package_name = "vla_robot_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vla_robot",
    maintainer_email="seungyong0284@gmail.com",
    description="그리퍼 카메라 노드",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "gripper_cam_node = vla_robot_vision.gripper_cam_node:main",
        ],
    },
)
