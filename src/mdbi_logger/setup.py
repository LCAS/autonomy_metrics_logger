from setuptools import find_packages, setup

package_name = 'mdbi_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibrahim Hroob',
    maintainer_email='ibrahim.hroub7@gmail.com',
    description='A centralized package provides a logging node designed to track and record all operational events of robots',
    license='Apache',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

