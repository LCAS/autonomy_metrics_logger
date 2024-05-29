from glob import glob
from setuptools import find_packages, setup

package_name = 'autonomy_metrics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim hroob',
    maintainer_email='ibrahim.hroub7@gmail.com',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metric_logger = autonomy_metrics.metric_logger:main',
        ],
    },
)
