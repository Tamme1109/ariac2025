from setuptools import find_packages, setup

package_name = 'sub'

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
    description='nodes for listening (and data collection) to advanced logical camera, rgb camera, and break beam sensor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = sub.sensor_listener:main',

            'rgbListener = sub.rgb_listener:main', 
            'collectData = sub.rgb_advanced_listener:main',      
            'break = sub.breakbeam_listener:main',        
            'rgbListener = sub.rgb_listener:main',             

            
        ],
    },
)
