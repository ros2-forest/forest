from setuptools import setup

package_name = 'cs_256_64b'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pc_sw_node = cs_256_64b.pc_sw_node:main',
		'send_img = cs_256_64b.send_img:main',
		'receive_img = cs_256_64b.receive_img:main',
        ],
    },
)
