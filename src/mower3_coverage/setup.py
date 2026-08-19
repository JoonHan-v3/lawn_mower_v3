from setuptools import find_packages, setup

package_name = 'mower3_coverage'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rockhan',
    maintainer_email='rockhan1122@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'boundary_loader.py = mower3_coverage.boundary_loader:main',
        'coverage_planner.py = mower3_coverage.coverage_planner:main',
        'coverage_executor.py = mower3_coverage.coverage_executor:main',
        'coverage_checker.py = mower3_coverage.coverage_checker:main',
        'path_tracking_monitor.py = mower3_coverage.path_tracking_monitor:main',
    ],
},
)
