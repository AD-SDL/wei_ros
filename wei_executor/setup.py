from setuptools import setup

package_name = 'wei_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rafael Vescovi',
    maintainer_email='ravescovi@anl.gov',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ 
        'console_scripts': [
            'wei_test_node = wei_executor.weiTestNode:main',
        ]
    },
)
