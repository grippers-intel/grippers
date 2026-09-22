"""설정 파일은 --symlink-install 로 빌드하면 소스를 직접 가리킨다.
teach_pose.py 가 소스의 arm_poses.yaml 을 고치면 재빌드 없이 반영된다."""
from glob import glob

from setuptools import setup

package_name = "vla_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vla_robot",
    maintainer_email="seungyong0284@gmail.com",
    description="vla_robot 런치와 설정",
    license="Proprietary",
)
