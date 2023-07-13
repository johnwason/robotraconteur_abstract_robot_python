from setuptools import setup, find_packages, find_namespace_packages

setup(
    name='robotraconteur_abstract_robot',
    version='0.2.1',
    description='Robot Raconteur Python Abstract Robot and Abstract Tool',
    author='John Wason',
    author_email='wason@wasontech.com',
    url='https://github.com/robotraconteur/robotraconteur_abstract_robot_python',
    package_dir={'': 'src'},
    packages=find_namespace_packages(where='src'),
    include_package_data=True,
    zip_safe=False,
    
    install_requires=[
        'RobotRaconteur',
        'RobotRaconteurCompanion',
        'numpy',
        'PyYAML',
        'setuptools',
        'importlib_resources',
        'general_robotics_toolbox',
        'scipy'
    ],
    tests_require=['pytest'],
    extras_require={
        'test': ['pytest']
    }
)