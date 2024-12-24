from numpy.typing import NDArray
from typing import Annotated, Any
# Аннотации для векторов различных размеров
Array7 = Annotated[NDArray[Any], (7,)]
Array6 = Annotated[NDArray[Any], (6,)] 
Array4 = Annotated[NDArray[Any], (4,)]
Array3 = Annotated[NDArray[Any], (3,)]
Array2 = Annotated[NDArray[Any], (2,)]
