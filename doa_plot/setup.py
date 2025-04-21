from setuptools import setup

package_name = 'doa_plot'

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
    description='DOA Plotter',
    license='LGP:',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'doa_plot = doa_plot.doa_plot:main',
            'theta_plot = doa_plot.theta_plot:main',
        ],
    },
)
