from setuptools import find_packages, setup

package_name = 'chapt5_python_tf'

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
    maintainer='zgzhou',
    maintainer_email='zhouzge@foxmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_tf_broadcaster_s = chapt5_python_tf.python_tf_broadcaster_s:main',
            'python_tf_broadcaster   = chapt5_python_tf.python_tf_broadcaster:main',
            'python_tf_listener      = chapt5_python_tf.python_tf_listener:main',
        ],
    },
)
