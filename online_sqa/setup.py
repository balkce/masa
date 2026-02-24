from setuptools import setup

package_name = 'online_sqa'
submodules = package_name+'/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balkce',
    maintainer_email='caleb.rascon@gmail.com',
    description='Online Speech Quality Assessment',
    license='LGP:',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'online_sqa = online_sqa.online_sqa:main',
            'online_sqa_exp = online_sqa.online_sqa_exp:main',
        ],
    },
)
