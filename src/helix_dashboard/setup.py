import os
from glob import glob
from setuptools import find_packages, setup

package_name = "helix_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install static files so they're accessible via get_package_share_directory
        (
            os.path.join("share", package_name, "static"),
            glob("helix_dashboard/static/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HELIX Dev",
    maintainer_email="dev@helix.local",
    description="HELIX Phase 4: real-time web dashboard.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "helix_dashboard_node = helix_dashboard.dashboard_node:main",
        ],
    },
)
