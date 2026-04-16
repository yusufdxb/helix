from setuptools import setup

package_name = 'helix_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='HELIX Phase 1 adapter layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helix_topic_rate_monitor = helix_adapter.topic_rate_monitor:main',
            'helix_json_state_parser = helix_adapter.json_state_parser:main',
            'helix_pose_drift_monitor = helix_adapter.pose_drift_monitor:main',
        ],
    },
)
