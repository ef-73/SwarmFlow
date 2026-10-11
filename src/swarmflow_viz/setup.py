from setuptools import find_packages, setup

package_name = "swarmflow_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/viz.launch.py"]),
        ("share/" + package_name + "/config", ["config/gazebo_gui.config"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SwarmFlow WS-E",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow marker publisher for Foxglove (design 11.1)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "viz_node = swarmflow_viz.viz_node:main",
            "tf_relay = swarmflow_viz.tf_relay:main",
        ],
    },
)
