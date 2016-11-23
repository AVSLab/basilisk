# updates the copyright information for all .cs files
# usage: call recursive_traversal, with the following parameters
# parent directory, old copyright text content, new copyright text content

import os

# add the list of directories to be excluded
excludedir = ["../License","../build","../External","../modules"]

def update_source(filename, oldcopyright, copyright):
    utfstr = chr(0xef)+chr(0xbb)+chr(0xbf)
    fdata = file(filename,"r+").read()
    isUTF = False
    knownFileType = False
    if (fdata.startswith(utfstr)):
        isUTF = True
        fdata = fdata[3:]

    if (filename.endswith(".c") or filename.endswith(".h") or filename.endswith(".cpp") or filename.endswith(".hpp") or filename.endswith(".i")  ):
        copyright    = "/*" + copyright + " */\n"
        oldcopyright = "/*" + oldcopyright + "*/\n"
        knownFileType = True
    if (filename.endswith(".py")  ):
        copyright    = "''' '''\n'''" + copyright + "'''\n"
        oldcopyright = "'''" + oldcopyright + "'''\n"
        knownFileType = True

    if (oldcopyright != None):
        if (fdata.startswith(oldcopyright)):
            fdata = fdata[len(oldcopyright):]

    if (knownFileType):
        if not (fdata.startswith(copyright)):
            print "updating "+filename
            fdata = copyright + fdata 
            if (isUTF):
                file(filename,"w").write(utfstr+fdata)
            else:
                file(filename,"w").write(fdata)

def recursive_traversal(dir,  oldcopyright, copyright):
    global excludedir
    fns = os.listdir(dir)
    print "listing "+dir
    for fn in fns:
        fullfn = os.path.join(dir,fn)
        if (fullfn in excludedir):
            continue
        if (os.path.isdir(fullfn)):
            recursive_traversal(fullfn, oldcopyright, copyright)
        else:
            if (fullfn.endswith(".c") or fullfn.endswith(".h") or fullfn.endswith(".cpp") or fullfn.endswith(".hpp") or fullfn.endswith(".i") or fullfn.endswith(".py")):
                update_source(fullfn, oldcopyright, copyright)

oldcright = file("licenseOld.txt","r+").read()
cright = file("license.txt","r+").read()
recursive_traversal("..", oldcright, cright)
exit()