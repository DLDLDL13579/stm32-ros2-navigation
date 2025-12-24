from setuptools import find_packages, setup

package_name = 'stm32_keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),

    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dl',
    maintainer_email='dl@todo.todo',
    description='Keyboard control for STM32 via serial port',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'keyboard_control_node = stm32_keyboard_control.keyboard_control_node:main',
        ],
    },
)
