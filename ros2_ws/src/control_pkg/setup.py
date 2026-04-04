from setuptools import find_packages, setup
from glob import glob
import os

package_name = "control_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ZHU WEIZHE",
    maintainer_email="your_email@example.com",
    description="Control package for ROS2vision target following and UNO serial bridging.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_follower_node = control_pkg.controllers.target_follower_node:main",
            "uno_serial_bridge_node = control_pkg.bridges.uno_serial_bridge_node:main",
        ],
    },
)