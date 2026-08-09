from setuptools import find_packages, setup

package_name = "pad_management"
data_files = []
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)

launch_files = [
    "launch/pad.launch.py",
    "launch/vicon.launch.py",
    "launch/lighthouse.launch.py",
    "launch/sim.launch.py",
]
for launch_file in launch_files:
    data_files.append(("share/" + package_name + "/launch", [launch_file]))

config_files = [
    "config/crazyflie_config_lighthouse.yaml",
    "config/crazyflie_config_vicon.yaml",
    "config/crazyflie_config_sitl.yaml",
    "config/crazyflie_sitl_container.yaml",
    "config/flies_config_lighthouse.yaml",
    "config/flies_config_vicon.yaml",
    "config/flies_config_sitl.yaml",
    "config/flies_config_sim.yaml",
    "config/pads_config_sim.yaml",
    "config/pads_config_vicon.yaml",
    "config/pads_config_sitl.yaml",
    "config/tracker_config.yaml",
    "config/webots_config.yaml",
]
for config_file in config_files:
    data_files.append(("share/" + package_name + "/config", [config_file]))


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="winni",
    maintainer_email="vinzenz@malke.info",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        "console_scripts": [
            "pad_broadcaster = pad_management.pad_broadcaster:main",
            "point_finder = pad_management.point_finder:main",
            "pointcloud_combiner = pad_management.point_cloud_combiner:main",
            "sitl_creator = pad_management.sitl_creator:main",
            "pad_creator = pad_management.pad_creator:main",
            "pad_land_circle = pad_management.pad_land_circle:main",
            "pad_spawner = pad_management.pad_spawner:main",
        ],
    },
)
