from setuptools import find_packages, setup

package_name = "swarmflow_payload"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/payload.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SwarmFlow WS-F",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow package pose-follower (design 7.4)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "payload_node = swarmflow_payload.payload_node:main",
        ],
    },
)
