from setuptools import setup

package_name = 'flood_fill_pkg'

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
    maintainer='lrs-ubuntu',
    maintainer_email='xmihalikj1@stuba.sk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "floodfill_node = flood_fill_pkg.FloodFillNode:main"
        ],
    },
)
