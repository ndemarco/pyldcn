#!/usr/bin/env python3
"""
pyldcn - Python LDCN Communication Library
Setup script
"""

from setuptools import setup, find_packages
import os

# Read long description from README
with open('README.md', 'r', encoding='utf-8') as f:
    long_description = f.read()

# Read version from package
version = {}
with open('pyldcn/__init__.py', 'r') as f:
    for line in f:
        if line.startswith('__version__'):
            exec(line, version)
            break

setup(
    name='pyldcn',
    version=version.get('__version__', '0.1.0'),
    author='NickyDoes',
    author_email='',
    description='Python library for Logosol LDCN communication',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/ndemarco/pyldcn',
    project_urls={
        'Bug Tracker': 'https://github.com/ndemarco/pyldcn/issues',
        'Documentation': 'https://github.com/ndemarco/pyldcn/tree/main/docs',
        'Source Code': 'https://github.com/ndemarco/pyldcn',
    },
    packages=find_packages(exclude=['tests', 'tests.*', 'docs', 'examples']),
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Manufacturing',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
        'License :: OSI Approved :: GNU General Public License v2 or later (GPLv2+)',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Operating System :: POSIX :: Linux',
    ],
    keywords='ldcn logosol cnc servo motion-control industrial-automation',
    python_requires='>=3.7',
    install_requires=[
        'pyserial>=3.4',
    ],
    extras_require={
        'dev': [
            'pytest>=6.0',
            'pytest-cov',
            'pylint',
            'black',
            'mypy',
        ],
    },
    entry_points={
        'console_scripts': [
            'device=pyldcn.cli.device:main',
        ],
    },
    include_package_data=True,
    zip_safe=False,
)
