from setuptools import find_packages, setup

package_name = 'aqlarp_motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'adafruit-pca9685-pip'],
    zip_safe=True,
    maintainer='AQLARP',
    maintainer_email='AQLARP@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aqlarp_motors = aqlarp_motors.aqlarp_motors:main',
            'calibration = aqlarp_motors.calibration:main'
        ],
    },
)
