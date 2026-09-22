from setuptools import find_packages, setup

package_name = "vla_robot_mission"

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
    description="Pi 미션 노드",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pi_mission_node = vla_robot_mission.pi_mission_node:main",
        ],
    },
)
