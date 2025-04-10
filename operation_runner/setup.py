from setuptools import find_packages, setup

package_name = 'py_srvcli'

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
    maintainer='louis',
    maintainer_email='louis@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_srvcli.server:main',
            'client = py_srvcli.client:main',
            'client2 = py_srvcli.client:main',
            'doorservice = py_srvcli.doorService:main',
            'doorClient = py_srvcli.doorClient:main',
            'json_service = py_srvcli.server_json:main',
            'json_client = py_srvcli.client_json:main',
            
        ],
    },
)
