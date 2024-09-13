from setuptools import setup, find_packages


def readme():
    with open('README.md', 'r', encoding='utf-8') as f:
        return f.read()


setup(
    name='pion',
    version='0.0.1',
    author='OnisOris',
    author_email='onisoris@yandex.ru',
    description='This module is needed to control drones.',
    long_description=readme(),
    long_description_content_type='text/markdown',
    url='https://github.com/OnisOris/pion',
    packages=find_packages(),
    install_requires=['numpy', 'pymavlink'],
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
