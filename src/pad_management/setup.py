from setuptools import find_packages, setup

package_name = "pad_management"
data_files = []
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)

data_files.append(("share/" + package_name + "/launch", ["launch/pad.launch.py"]))
data_files.append(("share/" + package_name + "/launch", ["launch/vicon.launch.py"]))
data_files.append(
    ("share/" + package_name + "/launch", ["launch/lighthouse.launch.py"])
)


data_files.append(
    ("share/" + package_name + "/config", ["config/flies_config_hardware.yaml"])
)
data_files.append(
    ("share/" + package_name + "/config", ["config/pads_config_hardware.yaml"])
)
data_files.append(("share/" + package_name + "/config", ["config/tracker_config.yaml"]))
data_files.append(("share/" + package_name + "/config", ["config/webots_config.yaml"]))

data_files.append(
    ("share/" + package_name + "/config", ["config/lighthouse_config.yaml"])
)
data_files.append((f"share/{package_name}/config", ["config/crazyflie_config_lh.yaml"]))
data_files.append((f"share/{package_name}/config", ["config/crazyflie_config_vicon.yaml"]))


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
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pad_broadcaster = pad_management.pad_broadcaster:main",
            "point_finder = pad_management.point_finder:main",
            "pointcloud_combiner = pad_management.point_cloud_combiner:main",
            "pad_destruction = pad_management.pad_destruction:main",
            "creator = pad_management.creator:main",
            "default_creator = pad_management.default_creator:main",
            "gui_creator = pad_management.gui_creator:main",
            "pad_creator = pad_management.pad_creator:main",
            "pad_traffic_controller = pad_management.pad_traffic_controller:main",
            "pad_land_circle = pad_management.pad_land_circle:main",
            "pad_spawner = pad_management.pad_spawner:main",
            "gui_state = pad_management.gui_state:main",
        ],
    },
)
