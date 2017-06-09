**Description:** Modular C/C++ astrodynamics simulation framework with Python scripting

**License:** [ISC Open Source License](https://en.wikipedia.org/wiki/ISC_license)

**Status:** Limited public alpha release


- - - - - - 

Architecture {#cover}
------------
The Basilisk astrodynamics software architecture is being designed to be 
capable of both faster-than realtime simulations, including repeatable 
Monte-Carlo simulation options, as well as providing real-time options for 
hardware-in-the-loop simulations. The Basilisk package is designed as a set of 
Python modules written in C/C++ which allows for the ease of scripting and 
reconfigurability of Python while still providing the execution speed of C/C++. 
The software is being developed jointly by the University of Colorado 
[AVS Lab](http://hanspeterschaub.info/AVSlab.html) and the [Laboratory for Atmospheric and 
Space Physics](http://lasp.colorado.edu/home/) (LASP). The resulting framework is targeted for both 
astrodynamics research modeling the orbit and attitue of complex spacecraft systems, as well as sophisticated mission-specific vehicle 
simulations that include hardware-in-the-loop scenarios.

\image html Images/bskImg1.png "Basilisk Concept Illustration" width=245px 

Name/Logo Description
---------------------
The name Basilisk was chosen to reflect both the reptilian (Python) 
nature of the product-design as well as a nod to the speed requirements as the 
South American common basilisk runs so fast that it can even run across water.  

Basilisk Design Goals
---------------------
The Basilisk framework is being designed from inception to support several different (often competing) requirements.

- **Speed:** Even though the system is operated through a Python 
    interface, the underlying simulation executes entirely in C/C++ which 
    allows for maximum execution speed.  For example, a goal is to simulate a mission year 
    with sufficiently accurate vehicle 6-DOF dynamics with at least a 365x  speed-up 
    (i.e. *a year in a day*).
    
- **Reconfiguration:** The user interface executes natively in 
    Python which allows the user to change task-rates, model/algorithm 
    parameters, and output options dynamically on the fly.
    
- **Analysis:** Python-standard analysis products like [numpy](http://www.numpy.org) and 
    [matplotlib](http://matplotlib.org) are actively used to facilitate rapid and complex analysis of 
    data obtained in a simulation run without having to stop and export to an 
    external tool.  This capability also applies to the Monte-Carlo engine 
    available natively in the Basilisk framework.
    
- **Hardware-in-the-Loop:** Basilisk will provide synchronization to realtime via software-based 
    clock tracking modules.  This allows the package to synchronize itself to 
    one or more timing frames in order to provide deterministic behavior in a 
    realtime environment.  External communication is handled via the [Boost](http://www.boost.org) 
    library with ethernet currently available and serial planned in the near 
    future.

- **Scriptability:** The Python user interface to the C/C++ layer relies on the [Simplified Wrapper 
    and Interface Generator](http://swig.org) (SWIG) software, a cross-platform, open-source 
    software tasked solely with interfacing C/C++ with scripting languages.  This Python layer 
    allows the simulation to be easily reconfigured which allows the user complete freedom in 
    creating their own simulation modules and flight software (FSW) algorithm modules.  Further, 
    the Python layer abstracts logging/analysis which allows a single compilation of the source 
    code to support completely different simulations. 

- **Controlled Data Flow:** Simulation modules and FSW algorithm modules communicate through 
    the message passing interface (MPI), which is a singleton pattern. The MPI allows data 
    traceability and ease of test. Modules are limited in their ability to subscribe to 
    messages and write messages, thus setting limitations on the flow of information and 
    the power of modules to control data generation.  The messaging system is also 
    instrumented to track data exchange, allowing the user to visualize exactly what data 
    movement occurred in a given simulation run.

- **Cross-Platform Solution:** Basilisk is inherently cross-platform in nature, and is 
    supported on macOS, Windows, and Linux systems.  The Python layer, C programming 
    language, Boost communication library and Qt/Open-GL visualization are active 
    cross-platform developments.

- **Validation and Verification:** Each simulation or FSW algorithm module has unit test 
    that can be run automatically using py.test.  Integrated scenario test validated 
    coupled behavior between modules.  Each dynamics modules has associated momentum, 
    energy and power validation tests.  This ensures the integrity of the valdiated 
    modules as new simulation capabilities are added.

- **Monte-Carlo Capability:** The simulation framework is capable of doing bit-for-bit 
    repeatable Monte-Carlo runs.  The simulation parameters can be disturbed through a 
    range of distribution functions.

- **3D Visualization:** Basilisk has an accompanying stand-alone visualization that 
    uses [Qt](https://www.qt.io)/[OpenGL](https://www.opengl.org) to visualize the 
    spacecraft, its orientation and orbits, the local planets, and various qualitative 
    data and indicators for sensors and actuators. Simulation events and device faults 
    may be triggered directly from the visualization.

