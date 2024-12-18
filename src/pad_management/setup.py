from setuptools import find_packages, setup

package_name = "pad_management"
data_files = []
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)

data_files.append(("share/" + package_name + "/launch", ["launch/pad.launch.py"]))
data_files.append(
    ("share/" + package_name + "/launch", ["launch/pad_example.launch.py"])
)

data_files.append(
    ("share/" + package_name + "/config", ["config/pad_arrangement.yaml"])
)
data_files.append(
    ("share/" + package_name + "/config", ["config/padflie_arrangement.yaml"])
)

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
            "pad_creator = pad_management.pad_creator:main",
        ],
    },
)
