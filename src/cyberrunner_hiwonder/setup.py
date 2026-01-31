from setuptools import find_packages, setup

package_name = 'cyberrunner_hiwonder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index resource so ROS can discover the package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trungbao',
    maintainer_email='trungbao@todo.todo',
    description='Hiwonder HID driver node for CyberRunner',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'hiwonder_node = cyberrunner_hiwonder.hiwonder_node:main',
    ],
},

)

