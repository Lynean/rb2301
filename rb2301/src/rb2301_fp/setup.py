from setuptools import find_packages, setup

package_name = 'rb2301_fp'

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
    maintainer='marmot',
    maintainer_email='marmot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    python_requires='>=3.8',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_course = rb2301_fp.obstacle_course:main',
            'obstacle_tracker = rb2301_fp.obstacle_tracker:main',
            'blindzone_check = rb2301_fp.blindzone_check:main',
        ],
    },
)
