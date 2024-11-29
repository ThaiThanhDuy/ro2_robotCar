from setuptools import setup

package_name = 'node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tf_publisher_launch.py']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
      
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_publisher = node.tf_publisher:main',  
            'twist_publisher = node.twist_publisher:main',  
            'stm32_interface = node.stm32_interface:main',  
            'joint_state_publisher = node.joint_state_publisher:main', 
        ],
    },
    
)