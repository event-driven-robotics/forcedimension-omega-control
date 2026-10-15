"""Setup script for omega_control package."""

from setuptools import setup, find_packages
import os

# Read the contents of README file
this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

# Read requirements
with open(os.path.join(this_directory, 'requirements.txt'), encoding='utf-8') as f:
    requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

setup(
    name='forcedimension-omega-control',
    version='1.0.0',
    description='Python control library for Force Dimension omega.x haptic devices',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Simon Mueller-Cleve, Quentin Halbach',
    author_email='simon.mullercleve@iit.it, halbach.quentin@iit.it',
    maintainer='Simon Mueller-Cleve',
    maintainer_email='simon.mullercleve@iit.it',
    url='https://github.com/event-driven-robotics/forcedimension-omega-control',
    license='GPL-3.0',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: POSIX :: Linux',
        'Operating System :: MacOS',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Programming Language :: Python :: 3.12',
        'Topic :: Scientific/Engineering :: Human Machine Interfaces',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    keywords='haptic, robotics, force-dimension, omega, control, sdk',
    python_requires='>=3.10,<3.13',
    packages=find_packages(),
    install_requires=requirements,
    extras_require={
        'dev': [
            'pytest>=7.0.0',
            'pytest-cov>=4.0.0',
            'black>=23.0.0',
            'flake8>=6.0.0',
            'mypy>=1.0.0',
        ],
    },
    entry_points={
        'console_scripts': [
            'omega-control=omega_control.cli:main',
            'omega-control-doctor=omega_control.doctor:main',
        ],
    },
    include_package_data=True,
    zip_safe=False,
    project_urls={
        'Bug Reports': 'https://github.com/event-driven-robotics/forcedimension-omega-control/issues',
        'Source': 'https://github.com/event-driven-robotics/forcedimension-omega-control',
        'Documentation': 'https://github.com/event-driven-robotics/forcedimension-omega-control/blob/main/README.md',
        'Institution': 'https://www.iit.it/',
    },
)