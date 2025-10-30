from setuptools import setup

package_name = 'video_stream'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@example.com',
    description='USB 카메라 영상 스트리밍 노드',
    license='MIT',
    entry_points={
        'console_scripts': [
            'video_stream_node = video_stream.video_stream_node:main',
            'rtsp_stream_node = video_stream.rtsp_stream_node:main',  # ✅ Jetson Orin용 추가
        ],
    },
)

