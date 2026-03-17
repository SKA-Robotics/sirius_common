from setuptools import setup

package_name = 'sirius_tty_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patryk Filip Gryz',
    maintainer_email='patryk.filip.gryz@rxsio.com',
    description='Sirius sensors communicating using TTY',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensors = sirius_tty_sensors.sensors:main',
        ],
    },
)