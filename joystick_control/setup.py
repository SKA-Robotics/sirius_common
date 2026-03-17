from setuptools import setup

package_name = 'joystick_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mateusz Frejlich',
    maintainer_email='mateuszfrejlich533@gmail.com',
    description='Steering the rover and the manipulator using joystick',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_diff_drive = joystick_control.joy_diff_drive:main',
            'joy_5dof_manipulator = joystick_control.joy_5dof_manipulator:main',
            'joy_multiplexer = joystick_control.joy_multiplexer:main',
        ],
    },
)