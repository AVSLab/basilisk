# Coding Guidelines {#codingGuidelines}

## Motivation##
Coding style is important. A clean, consistent style leads to code that is more readable, debuggable, and maintainable. To this end, we prescribe (and proscribe) a variety of practices. Our goal is to encourage agile but reasoned development of code that can be easily understood by others.

**C/C++ foundational guidelines**: this document uses as its foundation the coding guidelines developed by <a href="https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md"> Stroustrup and Sutter</a>.

**Python foundational guidelines**: the [PEP 8](https://www.python.org/dev/peps/pep-0008/) guidelines are to be followed for python code development.

The both of these foundational sets of guidelines are extensive and where the Basilisk document is silent these guidelines will prevail. 

These are guidelines, not rules. With very few exceptions, this document does not completely ban any particular C/C++ or Python pattern or feature, but rather describes best practice, to be used in the large majority of cases. When deviating from the guidelines given here, just be sure to consider your options carefully, and to document your reasoning, in the code.

Above all else, be consistent. Follow this guide whenever possible, but if you are editing a package written by someone else, follow the existing stylistic conventions in that package (unless you are retrofitting the whole package to follow this guide, for which you deserve an award).

## What about non-conforming code? 
Some Basilisk code was written prior to the release (and updates) of this style guide. Thus, the codebase may contain code that doesn't conform to this guide. The following advice is intended for the developer working with non-conforming code:

1. All new code should conform to this guide.
2. Unless you have copious free time, don't undertake converting the existing codebase to conform to this guide.
3. If you are the author of a non-conforming package, try to find time to update the code to conform.
4. If you are doing minor edits to non-conforming code, follow the existing stylistic conventions in that code (if any). Don't mix styles.
5. If you are doing major work on non-conforming code, take the opportunity to re-style it to conform to this guide.

## Naming ##
The following shortcuts are used in this document to denote naming schemes:

* CamelCased: The name starts with a capital letter, and has a capital letter for each new word, with no underscores.
* camelCased: Like CamelCase, but with a lower-case first letter
* under\_scored: The name uses only lower-case letters, with words separated by underscores.
* ALL\_CAPITALS: All capital letters, with words separated by underscores.

## General Guidelines

### Variables 
No single letter variables. The only exceptions are 'i' as an iteration index and a select list of mathematical symbols.

### Naming Variable for Mathematics 
The following section specifies general guidelines for the naming of variable to be used in code which implements mathematical operations. The naming convention is heavily influenced by the textbook *[Analytical Mechanics of Space Systems](http://arc.aiaa.org/doi/book/10.2514/4.102400)* by Schaub and Junkins.

### Indicating Reference Frames
A vector variable expressed with components in a reference frame \f$\cal B\f$,  is represented with the variable name followed by an underscore and a capital letter denoting the frame \f${}^{\cal B}\bf v\f$: `vector_B`

An angular rate variable expressed in one frame \f$\cal B\f$ with respect to a second \f$\cal R\f$, where components are expressed in the frame \f$\cal B\f$, \f${}^{\cal B}\pmb\omega_{\mathcal{B}/\mathcal{R}}\f$, is given as = `omega_BR_B`.

### MRP's and DCM's
A direction cosine matrix is expressed as \f$[BN]\f$, a mapping of an \f$\cal N\f$ frame vector into a \f$\cal B\f$ frame vector, is written `dcm_BN`. Similarly for the Modified Rodrigues Parameters (MRP) attitude parameterization the \f$\pmb\sigma_{\mathcal{B}/\mathcal{N}}\f$ is written sigma_BN. 

### Inertia Tensor
Inertia of the hub element with respect to point \f$BC\f$ defined in the body frame: `IHubPntBC_B`


### Derivatives
The first and second time derivatives of scalar (\f$\dot{x}\f$, \f$\ddot{x}\f$) or vector (\f$\dot{\bf{x}}\f$, \f$\ddot{\bf{x}}\f$) quantities, respectively are written as `xDot` and `xDDot`. 

The first and second time derivatives with respect to a variable other than time should use the same pattern as time derivatives but with a different modifier. For example, \f$f '(x)\f$ and \f$f ''(x)\f$ are written as `xPrime` and `xDPrime` respectively.

### Common Usage Examples
* Position vector from \f$\cal N\f$ to \f$\cal B\f$ in inertial frame components \f${}^{\cal N} \bf r_{\mathcal{B/N}}\f$: `r_BN_N`
* Inertial time derivative of position vector from \f$\cal N\f$ to \f$\cal B\f$ in inertial frame components \f${}^{\cal N} \dot{\bf r}_{\cal B/N}\f$: `rDot_BN_N`
* Time derivative with respect to the body of position vector from \f$ B\f$ to \f$ H\f$ in body frame components \f${}^{\cal B} \bf r'_{H/B}\f$: `rPrime_HB_B`
* Unit direction vector from \f$B\f$ to \f$S\f$ in body frame components \f${}^{\cal B} \hat{\bf s}_{S/B}\f$: `sHat_SB_B`
* Inertial time derivative of body angular rate with respect to the inertial frame in body frame components \f${}^{\cal B} \dot{\pmb\omega}_{\mathcal{B}/\mathcal{N}}\f$: `omegaDot_BN_B`
* DCM of the body frame with respect to the inertial frame \f$[BN]\f$: `dcm_BN`













## Modules ##
### Messages ###
Variables holding message names are to be composed in the following manner.

```{.cpp}
std::string sunEphmInMsgName;
```
* `sunEphm`: description of the message content.
* `In` (`Out`): indicates the direction of the message with respect to the module.
* `MsgName`: explicitly identifies the variable as a message name and is required for all message name variables.

Variables holding a message identification number are to be composed in the following manner.

```{.cpp}
int64_t stateInMsgId;
```
* `state`: description of the message content.
* `In` (`Out`): indicates the direction of the message with respect to the module.
* `MsgId`: explicitly identifies the variable as a message identifier and is required for all message identifier variables.

Variables holding data from a read message are to be composed in the following manner.

```{.cpp}
RWCmdStruct* rwCommandInBuffer;
```
* `rwCommand`: description of the data.
* `In` (`Out`): indicates the direction of the data being written to the buffer with respect to the module.
* `Buffer`: explicitly identifies the variable as having a data buffer functionality.

## C/C++ Exceptions
* Currently no language specific exceptions

## Python Exceptions
* Variables are to be camelCase. This is done to maintain consistency across the C/C++ and Python code bases which are interfaced via SWIG. 
* Inline comments are accepted so long as they are kept brief.
* Binary operator spaces will be adhered to as specified in PEP 8, however, not for math symbols operations. E.g. no spaces are included around *, /, +, -, etc 

```{.py}
# Yes
x = (4*9/2)-1
# No
x = (4 * 9 / 2) - 1
```

The following LaTeX markup is saved so that the above math variable section can be reproduced and updated as needed. Currently there is no LaTeX rendering in Bitbucket readme or wiki files.
