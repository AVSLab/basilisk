import glob
import os

def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        if f.readline() is ".. toctree::\n\n":
            pass
        else:
            f.seek(0,0)
            f.write(line.rstrip('\r\n') + '\n' + content)

def line_prepender_all(filename,files):
    if not os.path.exists(filename):
        f = open(filename, 'w')
        f.close()
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)

        # Check if the file already has a toctree
        if f.readline() is ".. toctree::\n\n":
            pass
        else:
            #If not, add one and label it with the sub-directory name
            f.seek(0,0)
            line = ".. toctree::\n"
            line += "   :maxdepth: 1\n"
            line += "   :caption: " + filename[:filename.rfind('/')] + ":\n\n"
            files = sorted(files)

            # Add the sub-directory files to the root dir .rst file
            for file in files:
                if "index" in file:
                    continue
                line += "   " + str(file[file.rfind('/')+1:file.rfind(".")]) + "\n"

            # Add a header too
            line += "\n\n"

            f.write(line.rstrip('\r\n') + '\n' + content)

def main():
    # Look in all directories you want a file structure
    dirs = ["Tutorial", "Install", "Support", "About", "Vizard"]

    # Grab all .rst files on root directory level
    rstFiles = glob.glob("*.rst")

    # For each directory, check if there is a .rst in the root directory
    for dir in dirs:

        # if no, make one and add the folder's index file
        if dir+".rst" not in rstFiles:
            with open(dir+".rst", 'w') as f:
                file = dir+".rst"
                line += file[:file.rfind('/')] + "\n"
                line += "=====================\n\n"

                line = ".. toctree::\n"
                line += "   :maxdepth: 1\n"
                line += "   :caption: " + file[:file.rfind('.')] + ":\n\n"

                line += file[:file.rfind('.')]+"/index\n\n"

            with open(dir + "/index.rst", 'w') as f:
                f.write("\n")

        #Grab all files within that sub-directory
        files = glob.glob(dir+"/*.rst")

        # Add all files in the sub-folder to subdirectory index.rst file
        line_prepender_all(dir+"/index.rst", files)

        # Add toctree to all files in the sub-folder
        for file in files:
            line_prepender(file, ".. toctree::\n\n")

main()
