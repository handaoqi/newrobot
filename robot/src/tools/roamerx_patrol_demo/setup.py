from glob import glob
import os

from setuptools import find_packages, setup


package_name = "roamerx_patrol_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Deterministic, read-only indoor patrol data source",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_publisher = roamerx_patrol_demo.patrol_publisher:main",
            "bag_contract = roamerx_patrol_demo.bag_contract:main",
        ],
    },
)
