"""vla_common — ROS 에 의존하지 않는 공용 모듈.

Pi 에서는 colcon 이 설치하고, Host(노트북)에서는 `pip install -e` 로 같은 소스를
그대로 쓴다. 계약 파일이 한 벌뿐이라 양쪽 규격이 갈라질 수 없다.
"""
from setuptools import find_packages, setup

package_name = "vla_common"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="vla_robot",
    maintainer_email="seungyong0284@gmail.com",
    description="Host/Pi 공용 계약",
    license="Proprietary",
    tests_require=["pytest"],
)
