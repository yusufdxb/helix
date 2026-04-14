from setuptools import setup

package_name = 'helix_core'

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
    description='HELIX fault sensing nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helix_heartbeat_monitor = helix_core.heartbeat_monitor:main',
            'helix_anomaly_detector = helix_core.anomaly_detector:main',
            'helix_log_parser = helix_core.log_parser:main',
        ],
    },
)
