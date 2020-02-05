from setuptools import setup

package_name = 'luos'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yoan Mollard',
    maintainer_email='yoan@aubrune.eu',
    description='Luos lets you easily connect, interact and program your robots in ROS 1 and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker = luos.broker:main'
        ],
    },
)
