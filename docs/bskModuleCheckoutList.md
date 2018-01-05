# Basilisk Module Checkout List

This documents contains a series of action items that should be checked before a Basilisk (BSK) module is approved.

## Style and Formating
* Do the code variables satisfy the [Basilisk code style guidelines](@ref codingGuidelines)?
* Are 4 spaces used instead of tabs?

## Module Programming
* If new message types are declared, are these also Swig'd in `simulation/simMessages/simMessages.i`, `simulation/simFswInterfaceMessages/simFswInterfaceMessages.i` or `fswAlgorithms/fswMessages/fswMessages.i`
* Are all module input and output messages Swig's in the module `*.i` file
* Does the code contain appropriate comments
* Does the code contain Doxygen compatible function descriptions, variable defintions, etc.
* Module startup and initialization
    * The `SelfInit()` routine should declare the module output messages
    * The `CrossInit()` routine should subscribe to the module input messages
    * The `Reset()` in the FSW modules should reset all the default moudle configuration parameters.

## Module Functionality Testing
Is a _UnitTest folder included that:

* has the python file name start with`test_`
* only uses the test module (if possible), and creates the various required input messages
* checks the module output for all input and module configuration conditions
* performs the validation checking against either custom-computed results in other programs (Matlab, Mathematica, hand calculation), live Python computed results, or against expected simulation data (consistency checking)
* can also be run with the python command instead of pytest (by updating the `__main()__` function at the bottom of the python file)
* if the module depends on other modules, clear out teh `*.pyc` files in `utilities/`
    
## Module Documentation
Is a _Documentation folder included that

* uses the BSK module template provided in `/fswAlgorithms/_fswTemplateFolder/fswModuleTemplate/_Documentation`
* populates the various module description sections including
    * overview of module functionality
    * testing and validation
    * user guide on how to use the module
* Does the module *.h file include a dOxygen statement near top that links in the module `_Documentation` file. 
* Is the `docs/DoxyData` file updated in the `HTML_EXTRA_FILES` tag to include the modules PDF documentation file?  This step copies this PDF into the `docs/html` folder when running dOxygen.