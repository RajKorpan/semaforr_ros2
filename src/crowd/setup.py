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
        (
            'share/' + package_name + '/config',
            ['config/crowd.yaml'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/social_diagnostics.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Guan',
    maintainer_email='ericguan04@gmail.com',
    description=(
        'ROS 2 crowd-grid diagnostics for the canonical SemaFORR social API.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crowd_model = semaforr_crowd.node:main',
            'crowd_learner = semaforr_crowd.node:main',
            'crowd_count = semaforr_crowd.node:main',
            'crowd_behavior = semaforr_crowd.node:main',
            'crowd_cusum = semaforr_crowd.node:main',
            'crowd_bayes_cusum = semaforr_crowd.node:main',
            'crowd_count_thompson = semaforr_crowd.node:main',
        ],
    },
)
