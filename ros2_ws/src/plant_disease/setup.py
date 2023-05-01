from setuptools import setup

package_name = 'plant_disease'

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
            'publisher = plant_disease.drone_publisher:main',
            'subscriber_1 = plant_disease.prediction_model:main',
            'subscriber_2 = plant_disease.extract_color:main',
        ],
    },
)
