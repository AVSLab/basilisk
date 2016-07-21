%include "std_vector.i"
%include "std_string.i"
%include "std_set.i"
%include "std_pair.i"
%include "swig_conly_data.i"

%array_functions(bool, boolArray);

// Instantiate templates used by example
namespace std {
   %template(IntVector) vector<int>;
   %template(DoubleVector) vector<double>;
   %template(StringVector) vector<string>;
   %template(StringSet) set<string>;
   %template(intSet) set<unsigned long>;
   %template(ConstCharVector) vector<const char*>;
}
