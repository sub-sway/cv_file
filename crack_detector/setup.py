from setuptools import setup

package_name = 'crack_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@example.com',
    description='균열 감지 YOLOv8 노드',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crack_detector_node = crack_detector.crack_detector_node:main',
        ],
    },
)

