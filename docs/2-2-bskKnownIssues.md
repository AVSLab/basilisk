# Basilisk Known Issues {#bskKnownIssues}


## Version 0.X.X
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
    </li>
</ul>

## Version 0.6.0
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
    </li>
</ul>

## Version 0.5.1
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
    </li>
</ul>

## Version 0.5.0
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
</li>
    <li>the `exponentialAtmosphere` module has been replaced with a module based on the new atmospheric density base class.  BSK simulations that used the older module must update to use the new module.  The module unit test scripts illustrate how to use this module, and the module PDF documentation discusses this as well.  The `dragEffector` integrated test is also updated to make use of the new module
    </li>
    <li>
    The `MRP_Feedback()` has the control vector `domega0` removed and keeps this term now as a permanent zero vector.  Any code that was setting this needs to be updated to not set this parameter anymore.
    </li>
    </ul>

## Version 0.4.1
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
</li>
        <li>
            The `numpy` python package can't be the current version 1.16.x as this causes some incompatibilities and massive amounts of depreciated warnings.  These warnings are not related to BSK python code, but other support code.  Thus, for now be sure to install version 1.15.14 of `numpy`.
        </li>
    </ul>
    


## Version 0.4.0
   <ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
</li>
        <li>Version 4.x.x and higher of pytest works again with Basilisk.  You are free to install the latest version of pytest.
        </li>
        <li>
            As we are now using the conan package management system, you can't double the the Cmake GUI application.  Instead, you must either launch the Cmake GUI application from the command line, or run CMake from the command line directly.  See the platform specific Basilisk installation instructions.
        </li>
        <li>
            The `numpy` python package can't be the current version 1.16.x as this causes some incompatibilities and massive amounts of depreciated warnings.  These warnings are not related to BSK python code, but other support code.  Thus, for now be sure to install version 1.15.14 of `numpy`.
        </li>
    </ul>
    
    
    
## Version 0.3.3
<ul>
        <li>WINDOWS ONLY: Windows users cannot currently run pytest directly on Basilisk `src/` directory (there will be non-resolved python path issues that will result in erroneous ImportErrors). Instead, to verify proper installation of Basilisk, windows users must enter the specific subdirectory they are attempting to test, only then to run pytest. This should result in appropriate behavior.  Right now there is no known solution to this issue.
</li>
        <li>The latest version of pytest (version 3.7.1) has a conflict with the RadiationPressure module unit test.  We are still investigating. In the meantime, using pytest version 3.6.1 is working correctly.  
        </li>
    </ul>