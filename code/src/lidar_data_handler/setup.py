from setuptools import setup, find_packages

package_name = 'lidar_data_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trolley_auto_drive_launch.py']),
    ],
    package_data={
        'lidar_data_handler': [
            'public/libs/linux/**/*',
            'public/libs/windows/**/*',
            'public/libs/mac_os/**/*',
            'public/libs/android/**/*',
        ],
    },
    # include_package_data=True使用MANIFEST.in文件,某些setuptools版本对**通配符支持不完整;两种方式互补，确保在不同环境都能工作
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k',
    maintainer_email='2407128277@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_data_handler = lidar_data_handler.receive_lidar_data:main'
        ],
    },
)
