from setuptools import find_packages, setup

package_name = "serial_reader"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Andrew Johnson",
    maintainer_email="andrewmjohnson549@gmail.com",
    description="Contains the a node for reading serial data and publishing to a topic",
    license="Apache",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_reader = serial_reader.publisher: main",
        ],
    },
)
