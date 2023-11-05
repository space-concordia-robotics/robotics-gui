from setuptools import find_packages, setup

package_name = "controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamad Ali Taha, Omar Ziad",
    maintainer_email="mohamadali.taha@spaceconcordia.ca, o.ziad@outlook.com",
    description="Space Concordia Robotics Rover GUI",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["gui = scripts.script:main"],
    },
)
