from setuptools import find_packages, setup


package_name = 'semaforr_crowd'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Guan',
    maintainer_email='ericguan04@gmail.com',
    description=(
        'Compatibility notice for crowd functionality migrated into SemaFORR.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crowd_model = semaforr_crowd.node:main',
        ],
    },
)
