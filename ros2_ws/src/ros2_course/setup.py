from setuptools import find_packages, setup

package_name = 'ros2_course'

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
    maintainer='ros_user',
    maintainer_email='ros_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = ros2_course.hello:main',
            'random_turtle = ros2_course.random_turtle:main',
            'chaser_turtle = ros2_course.chaser_turtle:main',
            'turtle_spin = ros2_course.turtle_spin:main',
            'fractal = ros2_course.fractal:main',
            'flower_turtle = ros2_course.flower_turtle:main',
            'turtle_spawner = ros2_course.turtle_spawner:main',
            'hunter_turtle = ros2_course.hunter_turtle:main',
            'draw_A = ros2_course.draw_A:main',
            'turtlesim_controller = ros2_course.turtlesim_controller:main'
        ],
    },
)
