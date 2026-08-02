from setuptools import find_packages, setup

package_name = "crazyflie_flight_controller_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/scaling.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="winni",
    maintainer_email="vinzenz@malke.info",
    description="Examples and scalability tests for the Crazyflie flight controller.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "scaling_controller = "
            "crazyflie_flight_controller_examples.scaling_controller:main",
        ],
    },
)
