from setuptools import find_packages, setup

package_name = "swarmflow_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="SwarmFlow lead",
    maintainer_email="lead@swarmflow.invalid",
    description="SwarmFlow orchestrator library (pure Python, design §6.7)",
    license="MIT",
    tests_require=["pytest"],
)
