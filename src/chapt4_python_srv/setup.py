from setuptools import find_packages, setup
from glob import glob
package_name = 'chapt4_python_srv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 将图片文件拷入 install 的资源文件夹中
        ('share/' + package_name+"/resource", ['resource/default.jpg','resource/pic1.jpg']),
        # 将 launch 文件拷入 install 的launch文件夹中
        ('share/' + package_name+"/launch", glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zgzhou',
    maintainer_email='zhouzge@foxmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detect_node = chapt4_python_srv.face_detect_node:main',
            'face_detect_client = chapt4_python_srv.face_detect_client:main',
        ],
    },
)
