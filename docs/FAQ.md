# FAQ - Frequency Asked Questions  {#FAQ}

The following Frequency Answer Questions are general and not operating specific.


**How do I run `pytest` to ensure all unit and integrated tests still pass**

* Answer: The following response assumes you have the Basilisk soure code.  You need to install the python utility `pytest` if this is not available on your system. Instructions are found at  \ref installOptionalPackages.  Next, navigate to the folder `Basilisk\src` and run `pytest` from there.

**How can I run `pytest` faster?**

* Answer: Glad you asked.  While Basilisk is a single threaded simulation, it is possible to run `pytest` in a multi-threaded manner.  
```
pip install --user pytest-xdist
```
After installing this utility you now run the multi-threaded version of `pytest` for 8 threads using
```
pytest -n 8
```

