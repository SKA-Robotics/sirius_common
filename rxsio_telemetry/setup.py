from setuptools import setup

package_name = 'rxsio_telemetry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patryk Filip Gryz',
    maintainer_email='patryk.filip.gryz@rxsio.com',
    description='The rxsio_telemetry package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'telemetry = rxsio_telemetry.telemetry:main',
        ],
    },
)