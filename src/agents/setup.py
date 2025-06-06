from setuptools import find_packages, setup

package_name = "agents"
data_files = []
data_files.append(("share/" + package_name, ["package.xml"]))
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name + "/launch", ["launch/agent.launch.py"]))
data_files.append(
    ("share/" + package_name + "/launch", ["launch/random_agent.launch.py"])
)
data_files.append(("share/" + package_name + "/launch", ["launch/swarm.launch.py"]))

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
            "random_walk_agent = agents.random_walk_agent:main",
            "wand_agent = agents.wand_agent:main",
            "agent = agents.agent:main"
        ],
    },
)
