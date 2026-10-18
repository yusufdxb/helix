import os
from glob import glob

from setuptools import setup

package_name = 'helix_arbiter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yusufdxb',
    maintainer_email='yusuf.a.guenena@gmail.com',
    description='HELIX motion arbitration layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helix_arbiter = helix_arbiter.arbiter_node:main',
            'helix_go2_sport_sink = helix_arbiter.go2_sport_sink:main',
            'helix_trace = helix_arbiter.trace:main',
            'helix_preflight = helix_arbiter.preflight:main',
            'helix_hw_stage = helix_arbiter.hw_stage:main',
            'helix_fake_go2 = helix_arbiter.fake_go2:main',
        ],
    },
)
