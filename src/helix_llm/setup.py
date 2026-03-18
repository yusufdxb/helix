from setuptools import find_packages, setup

package_name = "helix_llm"

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
    maintainer="HELIX Dev",
    maintainer_email="dev@helix.local",
    description="HELIX Phase 3: on-device LLM advisor.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "helix_llm_advisor = helix_llm.llm_advisor:main",
        ],
    },
)
