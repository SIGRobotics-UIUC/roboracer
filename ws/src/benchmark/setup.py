from glob import glob
from setuptools import find_packages, setup

setup(
    name='benchmark', version='0.2.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/benchmark']),
                ('share/benchmark', ['package.xml', 'README.md', 'VALIDATION.md']),
                ('share/benchmark/config', glob('config/*.yaml'))],
    package_data={'benchmark.analysis.modules': ['race_view.js']},
    install_requires=['setuptools'], zip_safe=True,
    maintainer='SIGRobotics', maintainer_email='sigrobotics@illinois.edu',
    description='Reproducible race collection and independent offline analysis.',
    license='MIT',
    entry_points={'console_scripts': [
        'benchmark = benchmark.cli:main',
        'benchmark_gate = benchmark.collector.gate:main',
        'benchmark_recorder = benchmark.collector.recorder:main']},
)
