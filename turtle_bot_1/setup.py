from setuptools import setup
from glob import glob

package_name = 'turtle_bot_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='l.murciac@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_bot_teleop = turtle_bot_1.turtle_bot_teleop:main',
            'turtle_bot_interface = turtle_bot_1.turtle_bot_interface:main',
            'turtle_bot_player = turtle_bot_1.turtle_bot_player:main'
        ],
    },
)
