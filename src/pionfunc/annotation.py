from typing import Annotated, Any

from numpy.typing import NDArray

Array17 = Annotated[NDArray[Any], (17,)]
"""
Annotated type hint for a numpy NDArray of any type with size 17.
"""

Array14 = Annotated[NDArray[Any], (14,)]
"""
Annotated type hint for a numpy NDArray of any type with size 14.
"""

Array7 = Annotated[NDArray[Any], (7,)]
"""
Annotated type hint for a numpy NDArray of any type with size 7.
"""

Array6 = Annotated[NDArray[Any], (6,)]
"""
Annotated type hint for a numpy NDArray of any type with size 6.
"""

Array4 = Annotated[NDArray[Any], (4,)]
"""
Annotated type hint for a numpy NDArray of any type with size 4.
"""

Array3 = Annotated[NDArray[Any], (3,)]
"""
Annotated type hint for a numpy NDArray of any type with size 3.
"""

Array2 = Annotated[NDArray[Any], (2,)]
"""
Annotated type hint for a numpy NDArray of any type with size 2.
"""
