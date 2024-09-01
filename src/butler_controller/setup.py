from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'butler_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),  # Include RViz config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Include config files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keshav',
    maintainer_email='keshavsridhar1999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_state_machine = butler_controller.custom_state_machine:main',
            'order_to_delivery_robot = butler_controller.order_to_delivery:main',
            'order_with_confirmations_robot = butler_controller.order_with_confirmation:main',
            'order_with_timeout_robot = butler_controller.order_with_timeout:main',
            'order_with_kitchen_return_robot = butler_controller.order_with_kitchen_return:main',
            'order_with_cancellation_robot = butler_controller.order_with_cancellation:main',
        ],
    },
)
