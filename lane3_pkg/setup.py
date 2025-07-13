from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lane3_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carol',
    maintainer_email='caroline1@kookmin.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane3_node = lane3_pkg.lane3_node:main',
            # 'control_node = control.control_node:main',
        ],
    },
)
