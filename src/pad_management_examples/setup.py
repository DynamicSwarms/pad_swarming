from setuptools import find_packages, setup

package_name = "pad_management_examples"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]

data_files.append(
    ("share/" + package_name + "/launch", ["launch/pad_right_test.launch.py"])
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
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pad_right_test = pad_management_examples.pad_right_test:main"
        ],
    },
)
