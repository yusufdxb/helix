from setuptools import find_packages, setup

package_name = "helix_recovery"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HELIX",
    maintainer_email="user@example.com",
    description="HELIX Phase 2 recovery engine",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "helix_recovery_planner = helix_recovery.recovery_planner:main",
        ],
    },
)
