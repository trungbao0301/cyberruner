from glob import glob
from setuptools import find_packages, setup

package_name = "cyberrunner_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="trungbao",
    maintainer_email="trungbao@todo.todo",
    description="CyberRunner vision + control pipeline",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node     = cyberrunner_vision.camera_node:main",
            "estimator_node  = cyberrunner_vision.estimator_node:main",
            "path_node       = cyberrunner_vision.path_node:main",
            "marble_node     = cyberrunner_vision.marble_node:main",
            "controller_node = cyberrunner_vision.controller_node:main",
            "tuner_node        = cyberrunner_vision.tuner_node:main",
            "calibration_node  = cyberrunner_vision.calibration_node:main",
        ],
    },
)
