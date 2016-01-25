%module pyswice
%{
    #include "../External/cspice/include/SpiceUsr.h"
    #include "../External/cspice/include/SpiceZpr.h"
%}

%ignore illumg_c;
%ignore prefix_c;
%ignore ekucei_c;
%ignore ekuced_c;
typedef char ConstSpiceChar;
typedef double SpiceDouble;
%include "../../External/cspice/include/SpiceZpr.h"
