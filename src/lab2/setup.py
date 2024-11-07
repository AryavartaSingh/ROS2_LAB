from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aryavarta',
    maintainer_email='aryavarta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample = lab2.sample:main',
            'publisher = lab2.publisher:main',
            'subscriber = lab2.subscriber:main',
            'subscriber_new = lab2.subscriber_new:main'
        ],
    },
)
