from setuptools import setup

package_name = 'physical_ai_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['simple_talker', 'simple_listener'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User Name',
    maintainer_email='user@example.com',
    description='Simple ROS 2 nodes for Physical AI chapter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_talker = simple_talker:main',
            'simple_listener = simple_listener:main',
        ],
    },
)