from glob import glob

from setuptools import find_packages, setup

package_name = "swarmflow_orchestrator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SwarmFlow WS-B",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow orchestrator: rclpy adapter around swarmflow_core FleetCore (design 6.5, 6.6)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = swarmflow_orchestrator.orchestrator_node:main",
        ],
    },
)
