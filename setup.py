from setuptools import setup, find_packages # , Extension
import numpy as np
from Cython.Distutils.extension import Extension

# Cython-модуль, который нужно скомпилировать
extensions = [
    Extension(
        name="pion.cython_pid",
        sources=["src/pion/cython_pid.pyx"],
        include_dirs=[np.get_include()],
        cython_directives={"language_level": "3str"},
    ),
]

setup(
    ext_modules=extensions,
)
