from setuptools import find_packages, setup

package_name = 'dogbot_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Long Liangmao',
    maintainer_email='foah@connect.hku.hk',
    description='ICM-20948 talker node for publishing imu messages.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_publisher = dogbot_imu.publisher:main',
        ],
    },
)
