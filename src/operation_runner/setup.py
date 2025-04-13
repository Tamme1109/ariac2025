from setuptools import find_packages, setup

package_name = 'operation_runner'

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
    maintainer='Louise Hasslöf',
    maintainer_email='hasslofl@chalmers.se',
    description='package contains a simple operation runner with operations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ops_run = operation_runner.control:main',
            'action_cli = operation_runner.client_used_during_development:main',
            
        ],
    },
)
