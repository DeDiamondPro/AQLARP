from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'aqlarp_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),

        (os.path.join('lib', package_name, 'src'), glob(package_name + '/src/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AQLARP',
    maintainer_email='AQLARP@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aqlarp_main = aqlarp_main.aqlarp_main:main'
        ],
    },
)
