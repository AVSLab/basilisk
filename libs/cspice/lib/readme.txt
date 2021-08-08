Instructions to download and build cspice library.


- Download the cspice.tar source code (https://naif.jpl.nasa.gov/naif/toolkit.html)

- edit src/cspice/inquire.c to add
	#include <unistd.h>

- to build a new library, remove all *.pgm files from within `src` folder

- in src/ sub-folders, remore `-m64` from `mkprodct.csh` files in `src`

- create an empty `lib` folder
