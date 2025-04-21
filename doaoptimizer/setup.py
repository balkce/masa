from setuptools import setup

package_name = 'doaoptimizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balkce',
    maintainer_email='caleb.rascon@gmail.com',
    description='Direction of Arrival Optimization by Maximizing Signal-to-Distortion',
    license='LGP:',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doaoptimizer = doaoptimizer.doaoptimizer:main',
            'doaoptimizer_fb = doaoptimizer.doaoptimizer_fb:main',
            'doaoptimizer_bc = doaoptimizer.doaoptimizer_bc:main',
        ],
    },
)
