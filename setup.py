from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize
import numpy as np

def readme():
    with open('README.md', 'r', encoding='utf-8') as f:
        return f.read()

# Cython-модуль, который нужно скомпилировать
extensions = [
    Extension(
        "pion.cython_pid",
        ["pion/cython_pid.pyx"],
        include_dirs=[np.get_include()]
    ),
]

setup(
    name='pion',
    version='0.0.1',
    ext_modules=cythonize(extensions),
    author='OnisOris',
    author_email='onisoris@yandex.ru',
    description='This module is needed to control drones.',
    long_description=readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/OnisOris/pion',
    packages=find_packages(),
    install_requires=['numpy', 'pymavlink', 'rich', 'paramiko'],
    classifiers=[
        'Programming Language :: Python :: 3.12',
        'License :: OSI Approved :: GNU General Public License v2 (GPLv2)',
        'Operating System :: OS Independent'
    ],
    keywords='drone quadrotor quadcopter control',
    project_urls={
        'GitHub': 'https://github.com/OnisOris/pion'
    },
    python_requires='>=3.9'
)
