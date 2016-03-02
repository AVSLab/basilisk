Instructions on how to use they Python script to automatically add the open source license text at the beginning of *.c, *.h, *.i, *.py, *.cpp, *.hpp files.


1. Make sure the file

	licenseOld.txt

contains the current license text.  When you run the script, this text will be removed from the files, and the new license included

2. Update the new license information in the file

	license.txt

3. Open a terminal window

4. change to the “License” sub-folder using the “cd” command

5. run the python script using

	python addLicense.py

The output will show you what folders are being included, and which files specifically are being updated.