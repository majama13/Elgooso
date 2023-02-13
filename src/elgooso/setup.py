import os
from glob import glob
from setuptools import setup

package_name = 'elgooso'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='majama',
    maintainer_email='majama@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'state_publisher = elgooso.state_publisher:main',
        	'move = elgooso.move:main',
        	'talker = elgooso.publisher_member_function:main',
        	'Qlearn = elgooso.Qlearn:main',
        	'spawn_entity = elgooso.spawn_entity:main',
        ],
    },
)
