from setuptools import find_packages, setup

package_name = 'csv_pkg'

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
    maintainer='ed',
    maintainer_email='ed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "logger_um7 = csv_pkg.save_um7_csv:main",
            "logger_fc = csv_pkg.save_fc_csv:main",
            "reader_um7 = csv_pkg.um7_imu_reader:main",
            "logger = csv_pkg.save_csv:main",
        ],
    },
)
