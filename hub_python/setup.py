import os
from glob import glob
from setuptools import setup

package_name = 'hub_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='lkw303@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hub_node=hub_python.hub_node:main',
            'test_deposit_service=hub_python.test_deposit_service:main',
            'test_deposit_action=hub_python.test_deposit_action:main',
            'test_collect_service=hub_python.test_collect_service:main',
            'test_collect_action=hub_python.test_collect_action:main',
        ],
    },
)
