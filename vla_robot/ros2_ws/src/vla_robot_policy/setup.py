from setuptools import find_packages, setup

package_name = "vla_robot_policy"

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
    description="VLA 파지 액션 노드",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "vla_grasp_node = vla_robot_policy.vla_grasp_node:main",
        ],
    },
)
