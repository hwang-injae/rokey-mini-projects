import os
from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = "sim_bringup"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/sim_bringup"]),
        (os.path.join("share", PACKAGE_NAME), ["package.xml"]),
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "config"),
            glob(os.path.join("config", "*.config")),
        ),
        (
            os.path.join("share", PACKAGE_NAME, "models"),
            glob(os.path.join("models", "*.sdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gesture_robot team",
    maintainer_email="team@example.com",
    description="Launch package for the gesture robot simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
