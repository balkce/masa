from setuptools import setup
import os
from glob import glob

package_name = 'masacoord'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balkce',
    maintainer_email='caleb.rascon@gmail.com',
    description='Multi-agent Auditory Scene Analysis coordinator.',
    license='LGP:',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'masacoord = masacoord.masacoord:main',
        ],
    },
)
