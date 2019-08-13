# Creating the HTML Basilisk Documentation using Doxygen


## Doxygen Documentation
The [Doxygen](http://doxygen.nl) software is an elegant method to both include code explanations, definitions and module documentation, but also to create a full HTML based documentation folder for a software project.  An online copy of this HTML documentation is hosted at [AVS Basilisk web site](http://hanspeterschaub.info/bskMain.html) with the [Documentation](http://hanspeterschaub.info/bskHtml/index.html) page.  
\image html Images/doc/bskHTML.png "BSK HTML Documentation Landing Page" width=80% 

The [Related Pages](http://hanspeterschaub.info/bskHtml/pages.html) tab provide a listing of a range of support files such as how to install Basilisk on Linux, macOS and Microsoft Windows. It also provides FAQ pages, BSK release notes, and other support files.

The [Modules](http://hanspeterschaub.info/bskHTML/modules.html) tab provides the listing of tutorial BSK scenario descriptions.  These are integrated BSK simulations written to be a series of tutorials to learn how to use BSK.

The [Classes](http://hanspeterschaub.info/bskHTML/annotated.html) tab provides access to the BSK software code documentation, include class and message header definitions.  Note that the class files contain interactive SVG images that make it convenient to jump between different documentation pages.


## Requirements to Create Local Copy of  Doxygen HTML Documentation
You either need to have a command line version of Doxygen installed or your system, or have downloaded a Doxygen GUI for your operating system.  

The Doxygen [download page](http://www.stack.nl/~dimitri/doxygen/download.html) contains a range of pre-compiled binaries for many different platforms.

On macOS the [Homebrew](https://brew.sh) tool is also a very convenient method to install Doxygen by typing in the terminal 
```
brew install doxygen
```

The [Doxygen Download page](http://www.stack.nl/~dimitri/doxygen/download.html) also contains links to download a Windows or macOS GUI to run Doxygen.

**WARNING**: The BSK Doxygen setup file has the flab `HAVE_DOT` set to true.  This allows for the HTML class description images to be vector SVG images.  Further, these images are interactive and embed hyperlinks to other related BSK classes.  As a result, make sure that the [GraphViz](http://www.graphviz.org) tool is installed and availabe in the default path.  On macOS using Homebrew you can simply type  
```
brew install graphviz
```
to install this tool.


## Creating HTML Documentation
To create the HTML documentation with all the associated scenario figures, be sure to run `pytest` first from within the `/src` directory.  Don't use the `pytest -n 4` multi-threaded version as the image generation requires in places a sequence of unit test to be run in a particular order.

The Basilisk Doxygen configuration file is stored in `docs/DoxyData`.  If the `doxygen` command line tool is installed then simply run 
```
doxygen DoxyData
```
to create the `html` documentation folder within the `docs` folder.  The primary web page is `docs/html/index.html`.  


To use the Doxygen GUI tools, open the Doxygen application and select `docs/DoxyData` as the Doxygen configuration file.  Next, select the `Run` tab and press the button `Run doxygen`.  When it completes the button `Show HTML output` is selectable and will launch the BSK documentation landing page in your web browser.
\image html Images/doc/bskDoxygen.png "macOS Doxgyen GUI" width=80% 


