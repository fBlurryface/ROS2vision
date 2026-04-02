from setuptools import find_packages, setup
from glob import glob
import os

package_name = "recognition_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fBlurryface",
    maintainer_email="fblurryface@example.com",
    description="Recognition package for ROS2vision image preprocessing and future inference nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_preprocessor_node = recognition_pkg.image_preprocessor_node:main",
        ],
    },
)