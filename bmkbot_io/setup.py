from setuptools import find_packages, setup

package_name = 'bmkbot_io'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 
        ('share/' + package_name, ['launch/bmkbot_io.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woosung',
    maintainer_email='woosung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_server = bmkbot_io.tts_server:main',
        ],
    },
)
