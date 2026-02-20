from setuptools import setup

package_name = 'f1tenth_gap_following'

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
    description='f1tenth gap_follow lab',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = f1tenth_gap_following.reactive_node:main',
             'plot_ranges   = f1tenth_gap_following.plot_ranges:main',
        ],
    },
)
