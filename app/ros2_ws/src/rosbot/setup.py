from setuptools import find_packages, setup

package_name = 'rosbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ieeeras',
    maintainer_email='ieeeras@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motor_controller = rosbot.motor_controller:main',
        'tf_publish = rosbot.tf_transform_odom_base:main'
        ],
    },
)
