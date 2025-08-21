from setuptools import find_packages, setup

package_name = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pytz'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A utility package for shared functionalities for MV Bot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
