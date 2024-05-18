from setuptools import find_packages, setup

package_name = 'ros-rpirobot-driver'

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
    maintainer='geri',
    maintainer_email='gergely.koloszar@gmail.com',
    description='USART interface for custom robot hardware',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = py_ctrl_interface.interface:main',
            'echo = py_ctrl_interface.test_echo:main',
        ],
    },
)
