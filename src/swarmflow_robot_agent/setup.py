from setuptools import find_packages, setup

package_name = "swarmflow_robot_agent"

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
    maintainer="SwarmFlow WS-D",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow robot agent: fleet route to Nav2 NavigateThroughPoses bridge (design 6.3)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent_node = swarmflow_robot_agent.agent_node:main",
            "fake_nav2 = swarmflow_robot_agent.fake_nav2:main",
        ],
    },
)
