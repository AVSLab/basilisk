%module swig_std_array

%include "architecture/utilities/bskException.swg"
%default_bsk_exception();


%include "std_array.i"

%define STD_ARRAY(type)

// Poor man's for loop
%template() std::array<type, 1>;
%template() std::array<type, 2>;
%template() std::array<type, 3>;
%template() std::array<type, 4>;
%template() std::array<type, 5>;
%template() std::array<type, 6>;
%template() std::array<type, 7>;
%template() std::array<type, 8>;
%template() std::array<type, 9>;
%template() std::array<type, 10>;

%enddef

STD_ARRAY(float)
STD_ARRAY(double)
STD_ARRAY(int)
STD_ARRAY(long)
