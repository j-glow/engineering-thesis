from setuptools import find_packages, setup

package_name = 'nodes'

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
    maintainer='j.glowacki',
    maintainer_email='jakubglowacki3@gmail.com',
    description='Package containing ROS-based nodes with logic regarding the engineering thesis project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
