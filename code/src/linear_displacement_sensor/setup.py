from setuptools import setup, find_packages

package_name = 'linear_displacement_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        'linear_displacement_sensor.public': ['libs/**/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k',
    maintainer_email='2407128277@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'left_sensor = linear_displacement_sensor.left_sensor:main',
            'right_sensor = linear_displacement_sensor.right_sensor:main',
        ],
    },
)
