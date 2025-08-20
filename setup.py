from distutils.core import setup
 
setup(
    version='0.0.0',
    scripts=['src/plot_path.py'],
    packages=['iris_control'],
    package_dir={'': 'src'},
    entry_points={
        'console_scripts': [
            'plot_path = iris_control.plot_path:main',
        ],
    })
