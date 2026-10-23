from glob import glob

from setuptools import find_packages, setup

package_name = 'ugp_warehouse'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amara',
    maintainer_email='you@example.com',
    description='UGP resilient multi-robot warehouse logistics',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'drive_test = ugp_warehouse.drive_test:main',
            'astar_nav = ugp_warehouse.astar_nav:main',
            'coordinator = ugp_warehouse.coordinator:main',
            'tf_relay = ugp_warehouse.tf_relay:main',
            'task_manager = ugp_warehouse.task_manager:main',
        ],
    },
)
