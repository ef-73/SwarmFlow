from setuptools import find_packages, setup

package_name = "swarmflow_scenarios"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/scenario_engine.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SwarmFlow WS-F",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow scenario_engine: publishes seeded orders in sim time and writes the run record (design 13.5)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario_engine = swarmflow_scenarios.scenario_engine:main",
        ],
    },
)
