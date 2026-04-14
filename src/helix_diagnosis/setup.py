from setuptools import setup

package_name = 'helix_diagnosis'

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
    description='HELIX diagnosis tier',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helix_context_buffer = helix_diagnosis.context_buffer:main',
            'helix_diagnosis_node = helix_diagnosis.diagnosis_node:main',
        ],
    },
)
