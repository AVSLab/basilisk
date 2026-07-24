# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder.
#
# This file does only contain a selection of the most common options. For a
# full list see the documentation:
# http://www.sphinx-doc.org/en/master/config

# -- Path setup --------------------------------------------------------------

import datetime
import shutil
# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath("_ext"))

benchmarkSourceRoot = os.path.abspath("../../benchmarks")
if os.path.isdir(benchmarkSourceRoot):
    for dirPath, _, fileNames in os.walk(benchmarkSourceRoot):
        if any(fileName.endswith(".py") for fileName in fileNames):
            sys.path.insert(0, dirPath)

import numpy as np

from docutils import nodes
from docutils.parsers.rst import roles

def beta_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    label = nodes.inline(rawtext, "[BETA]", classes=['beta-label'])
    return [label, nodes.Text(f" {text}")], []

roles.register_local_role('beta', beta_role)

def module_type_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    module_type = text.strip()
    css_class = module_type.lower().replace("+", "p")
    node = nodes.inline(
        rawtext,
        module_type,
        classes=['module-type-label', f'module-type-{css_class}']
    )
    return [node], []

roles.register_local_role('module-type', module_type_role)

#
# create RST file showing supportData folder information
#
from collections import defaultdict
from pathlib import Path, PurePosixPath


def _normalize_single_page_docname(doc_path):
    if not doc_path:
        return ""

    source_dir = Path(__file__).resolve().parent
    doc_path = doc_path.strip()
    candidate = Path(doc_path)
    if candidate.suffix == "":
        candidate = candidate.with_suffix(".rst")
    if candidate.suffix != ".rst":
        return ""

    if candidate.is_absolute():
        try:
            candidate = candidate.resolve(strict=False).relative_to(source_dir)
        except ValueError:
            return ""
    elif candidate.parts and candidate.parts[0] == "source":
        candidate = Path(*candidate.parts[1:])

    return candidate.with_suffix("").as_posix().lstrip("./")


def _single_page_docname_from_cli():
    rst_args = [arg for arg in sys.argv[1:] if arg.endswith(".rst")]
    if len(rst_args) != 1:
        return ""
    return _normalize_single_page_docname(rst_args[0])


single_page_docname = _normalize_single_page_docname(
    os.environ.get("BSK_DOCS_SINGLE_PAGE", "")
) or _single_page_docname_from_cli()

if single_page_docname:
    print(f"Basilisk docs single-page mode: {single_page_docname}")


def write_text_if_changed(output_file, contents):
    output_path = Path(output_file)
    if output_path.exists() and output_path.read_text(encoding="utf8") == contents:
        return
    output_path.write_text(contents, encoding="utf8")


from Basilisk.utilities.supportDataTools.dataFetcher import POOCH
def build_supportdata_index_rst(output_file: str):
    groups = defaultdict(list)

    for key in POOCH.registry.keys():
        p = PurePosixPath(key)

        # Expect: supportData/<topic>/<file>
        if len(p.parts) < 3 or p.parts[0] != "supportData":
            continue

        topic = p.parts[1]
        filename = p.name
        groups[topic].append(filename)

    # Sort topics and files
    for topic in groups:
        groups[topic].sort()

    lines = []
    lines.append(".. _supportDataList:")
    lines.append("")
    lines.append("Support Data Files")
    lines.append("==================")
    lines.append(".. note::\n")
    lines.append("   Below is a list of all the data files that are packaged into Basilisk.")
    lines.append("   They are organized into a range of topical folders where the folder name is the data category.")
    lines.append("")

    for topic in sorted(groups.keys()):
        title = topic.replace("_", " ").title()

        lines.append(title)
        lines.append("-" * len(title))
        lines.append("")

        for fname in groups[topic]:
            lines.append(f"- ``{fname}``")

        lines.append("")

    write_text_if_changed(output_file, "\n".join(lines))

output_file = "supportData.rst"
if not single_page_docname or single_page_docname == "supportData":
    build_supportdata_index_rst(output_file)


# -- Project information -----------------------------------------------------

now = datetime.datetime.now()
f = open('bskVersion.txt', 'r')
bskVersion = f.read()
f.close()

project = u'Basilisk'
copyright = str(now.year) + u', Autonomous Vehicle Systems (AVS) Laboratory'
author = u'AVS Lab'
release = bskVersion
version = u'version ' + release

from Basilisk.utilities.deprecated import BSKDeprecationWarning
import warnings
warnings.filterwarnings("ignore", category=BSKDeprecationWarning)

# -- General configuration ---------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
needs_sphinx = '7.0'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.graphviz',
    "sphinx_rtd_theme",
    'recommonmark',
    'breathe',
    'sphinx_copybutton',
    'bsk_module_io',
    'sphinxcontrib.youtube'
]

graphviz_output_format = "svg"
_dot_candidates = ["/opt/homebrew/bin/dot", "/usr/local/bin/dot"]
graphviz_dot = shutil.which("dot") or next((p for p in _dot_candidates if os.path.exists(p)), "dot")

# Use MathJax SVG output instead of CHTML to avoid browser/font-metric
# rendering artifacts (e.g., vertically offset multi-character symbols).
mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"
mathjax3_config = {
    "svg": {"fontCache": "global"}
}

# filter out terminal prompt text from the code blocks
copybutton_prompt_text = r"\(\.venv\) \$ |\$ "
copybutton_prompt_is_regexp = True

breathe_doxygen_config_options = {
    'WARN_AS_ERROR': 'YES'
    , 'WARN_IF_UNDOCUMENTED': 'YES'  # Ensure undocumented variables, functions, etc., raise warnings
}

# Add any paths that contain templates here, relative to this directory.
#templates_path = ['_templates']

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
#source_suffix = ['.rst', '.md', '.svg']
source_suffix = {'.rst': 'restructuredtext'}

# The master toctree document.
master_doc = single_page_docname or 'index'

if single_page_docname:
    include_patterns = [single_page_docname + ".rst"]

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = "en"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = [
    'examples/BskSim/scenarios/index.rst',
    'examples/BskSim/index.rst',
    'examples/MultiSatBskSim/scenariosMultiSat/index.rst',
    'examples/MultiSatBskSim/index.rst',
    'examples/MultiSatBskSim/tleData/index.rst',
    'examples/OpNavScenarios/scenariosOpNav/index.rst',
    'examples/OpNavScenarios/scenariosOpNav/CNN_ImageGen/index.rst',
    'examples/OpNavScenarios/scenariosOpNav/OpNavMC/index.rst',
    'examples/OpNavScenarios/index.rst',
    'examples/mujoco/index.rst',
    # Release note snippets are consumed via include in bskReleaseNotes.rst and
    # should not be treated as standalone documentation pages.
    'Support/bskReleaseNotesSnippets/*.rst',
    'Support/bskReleaseNotesSnippets/*.md',
]

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = None


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
html_theme_options = {
    'logo_only': False,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'vcs_pageview_mode': '',
    'style_nav_header_background': '#CFB87C',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

html_css_files = [
    'css/custom.css',
]

html_logo = "./_images/static/Basilisk-Logo.png"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.



#
# The default sidebars (for documents that don't match any pattern) are
# defined by theme itself.  Builtin themes are using these templates by
# default: ``['localtoc.html', 'relations.html', 'sourcelink.html',
# 'searchbox.html']``.
#
# html_sidebars = {}


# -- Options for HTMLHelp output ---------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'Basiliskdoc'


# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'Basilisk.tex', u'Basilisk Documentation',
     u'AVS Lab', 'manual'),
]


# -- Options for manual page output ------------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    (master_doc, 'basilisk', u'Basilisk Documentation',
     [author], 1)
]


# -- Options for Texinfo output ----------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    (master_doc, 'Basilisk', u'Basilisk Documentation',
     author, 'Basilisk', 'One line description of project.',
     'Miscellaneous'),
]


# -- Options for Epub output -------------------------------------------------

# Bibliographic Dublin Core info.
epub_title = project

# The unique identifier of the text. This can be a ISBN number
# or the project homepage.
#
# epub_identifier = ''

# A unique identification for the text.
#
# epub_uid = ''

# A list of files that should not be packed into the epub file.
epub_exclude_files = ['search.html']


# -- Extension configuration -------------------------------------------------


# breathe_projects = {"Basilisk": "../../src/*"}

from glob import glob

class fileCrawler():
    def __init__(self, newFiles=False, rust_header_dir=None):
        self.newFiles = newFiles
        self.breathe_projects_source = {}
        self.counter = 0
        self.rust_header_dir = Path(rust_header_dir).resolve() if rust_header_dir else None

    def grabRelevantFiles(self,dir_path):
        dirs_in_dir = glob(dir_path + '*/')
        files_in_dir = glob(dir_path + "*.h")
        files_in_dir.extend(glob(dir_path + "*.hpp"))
        files_in_dir.extend(glob(dir_path + "*.c"))
        files_in_dir.extend(glob(dir_path + "*.cpp"))
        files_in_dir.extend(glob(dir_path + "*.py"))
        files_in_dir.extend(glob(dir_path + "Cargo.toml"))


        # Remove any directories that shouldn't be added directly to the website
        removeList = []
        for i in range(len(dirs_in_dir)):
            if "_Documentation" in dirs_in_dir[i] or \
                    "__pycache__" in dirs_in_dir[i] or \
                    "_VizFiles" in dirs_in_dir[i] or \
                    "Support" in dirs_in_dir[i] or \
                    "cmake" in dirs_in_dir[i] or \
                    "benchmarks" in dirs_in_dir[i] or \
                    "topLevelModules" in dirs_in_dir[i] or \
                    "outputFiles" in dirs_in_dir[i] or \
                    "msgAutoSource" in dirs_in_dir[i] or \
                    "alg_contain" in dirs_in_dir[i] or \
                    "dataForExamples" in dirs_in_dir[i] or \
                    "tests" in dirs_in_dir[i]:
                removeList.extend([i])
        for i in sorted(removeList, reverse=True):
            del dirs_in_dir[i]

        # Remove unnecessary source files (all files except .py, .c, .cpp, .h)
        removeList = []
        for i in range(len(files_in_dir)):
            if "__init__" in files_in_dir[i] or \
                    (files_in_dir[i].endswith(".py") and os.path.basename(files_in_dir[i]).startswith("_")) or \
                    "conftest.py" in files_in_dir[i] or \
                    "*.xml" in files_in_dir[i] or \
                    "vizMessage.pb.cc" in files_in_dir[i] or \
                    "vizMessage.pb.h" in files_in_dir[i] or \
                    "vizMessage.proto" in files_in_dir[i] or \
                    "EGM9615.h" in files_in_dir[i] or \
                    "SunLineKF_test_utilities.py" in files_in_dir[i] or \
                    "datashader_utilities.py" in files_in_dir[i] or \
                    "reportconf.py" in files_in_dir[i]:
                removeList.extend([i])
        for i in sorted(removeList, reverse=True):
            del files_in_dir[i]

        paths_in_dir = []
        paths_in_dir.extend(dirs_in_dir)
        paths_in_dir.extend(files_in_dir)

        return paths_in_dir

    def seperateFilesAndDirs(self,paths):
        files = []
        dirs = []
        for path in paths:
            if os.path.isfile(path):
                files.append(path)
            elif os.path.isdir(path):
                dirs.append(path)
        return sorted(files), sorted(dirs)

    def getModuleType(self, module_files):
        file_extensions = {os.path.splitext(file_name)[1] for file_name in module_files}
        if ".cpp" in file_extensions or ".hpp" in file_extensions:
            return "C++"
        return "C"

    def _sourceRelativePath(self, path):
        return os.path.relpath(os.path.abspath(path), os.path.abspath(officialSrc)).replace(os.sep, "/")

    def _isSupportedBskModulePath(self, rel_path):
        return rel_path.startswith(("fswAlgorithms/", "simulation/", "moduleTemplates/"))

    def _hasMatchingModuleSource(self, src_path, module_name):
        return any(
            os.path.isfile(os.path.join(src_path, module_name + extension))
            for extension in (".c", ".cpp", ".h", ".hpp")
        )

    def isBskCppOrCModule(self, src_path, module_name):
        rel_path = self._sourceRelativePath(src_path)
        path_parts = rel_path.split("/")

        if not self._isSupportedBskModulePath(rel_path):
            return False
        if "_GeneralModuleFiles" in path_parts:
            if not rel_path.startswith(("fswAlgorithms/", "simulation/")):
                return False
            return os.path.isfile(os.path.join(src_path, module_name + ".rst")) and \
                self._hasMatchingModuleSource(src_path, module_name)

        return os.path.isfile(os.path.join(src_path, module_name + ".i"))

    def isBskPythonModule(self, py_file):
        module_name = os.path.splitext(os.path.basename(py_file))[0]
        parent_name = os.path.basename(os.path.dirname(py_file))
        rel_path = self._sourceRelativePath(py_file)
        path_parts = rel_path.split("/")

        if not rel_path.startswith(("fswAlgorithms/", "simulation/")):
            return False
        if any(folder in path_parts for folder in ("_UnitTest", "_Documentation", "_GeneralModuleFiles", "__pycache__")):
            return False
        if module_name != parent_name:
            return False

        return not os.path.isfile(os.path.join(os.path.dirname(py_file), module_name + ".i"))

    def isBskRustModule(self, manifest_file):
        src_path = os.path.dirname(manifest_file)
        module_name = os.path.basename(src_path)
        rel_path = self._sourceRelativePath(src_path)

        if not self._isSupportedBskModulePath(rel_path):
            return False

        return os.path.isfile(os.path.join(src_path, module_name + ".rst"))

    def _isCargoSourceDirectory(self, directory, file_paths):
        """Return whether *directory* is a Cargo crate's internal source folder."""
        if os.path.basename(os.path.normpath(directory)) != "src":
            return False

        return any(
            os.path.basename(file_path) == "Cargo.toml"
            for file_path in file_paths
        )

    def _isCargoTargetDirectory(self, directory, file_paths):
        """Return whether *directory* contains Cargo-generated build products."""
        if os.path.basename(os.path.normpath(directory)) != "target":
            return False

        return any(
            os.path.basename(file_path) == "Cargo.toml"
            for file_path in file_paths
        )

    def _generated_rust_header(self, module_name):
        """Return the generated module header when a Rust build produced it."""
        if self.rust_header_dir is None:
            return None

        header_path = self.rust_header_dir / f"{module_name}.h"
        return header_path if header_path.is_file() else None

    def writeModuleTitle(self, title_text, module_type=None):
        if module_type:
            title = f":module-type:`{module_type}` {title_text}"
        else:
            title = title_text
        return title + "\n" + "=" * len(title) + "\n\n"

    def populateDocIndex(self, index_path, file_paths, dir_paths):

        # get the folder name
        name = os.path.basename(os.path.normpath(index_path))
        lines = ""

        # if a _default.rst file exists in a folder, then use it to generate the index.rst file
        try:
            pathToFolder, folderName = dir_paths[0].split(name)
            docFileName = os.path.join(os.path.join(pathToFolder, name),  '_default.rst')
            with open(docFileName, 'r') as docFile:
                docContents = docFile.read()
            lines += docContents + "\n\n"

        except: # Auto-generate the index.rst file
            # add page tag
            normalizedIndexPath = index_path.replace(os.sep, "/").rstrip("/")
            if name.startswith("_"):
                pathToFolder = index_path.split("/"+name)[0]
                lines += ".. " + name + pathToFolder.split("/")[-1] + ":\n\n"
            elif name == 'utilities':
                pathToFolder = index_path.split("/" + name)[0]
                lines += ".. _Folder_" + name + pathToFolder.split("/")[-1] + ":\n\n"
            else:
                lines += ".. _Folder_" + name + ":\n\n"

            # Title the page
            lines += name + "\n" + "=" * len(name) + "\n\n"

            # pull in folder _doc.rst file if it exists
            try:
                pathToFolder, folderName = dir_paths[0].split(name)
                docFileName = os.path.join(os.path.join(pathToFolder, name), '_doc.rst')
                if os.path.isfile(docFileName):
                    with open(docFileName, 'r') as docFile:
                        docContents = docFile.read()
                    lines += docContents + "\n\n"
            except:
                pass

            # Add a linking point to all local files
            lines += """\n\n.. toctree::\n   :maxdepth: 1\n   :caption: """ + "Files:\n\n"
            calledNames = []
            rust_module_name = None
            for file_path in file_paths:
                if os.path.basename(file_path) != "Cargo.toml":
                    continue
                source_path = os.path.dirname(file_path)
                candidate_name = os.path.basename(source_path)
                if self.isBskRustModule(file_path):
                    rust_module_name = candidate_name
                    lines += "   " + rust_module_name + "\n"
                    calledNames.append(rust_module_name)
                    break
            for file_path in sorted(file_paths):
                fileName = os.path.basename(os.path.normpath(file_path))
                if fileName == "Cargo.toml":
                    continue
                fileName = fileName[:fileName.rfind('.')]
                if not fileName in calledNames:
                    lines += "   " + fileName + "\n"
                    calledNames.append(fileName)

            # Add a linking point to all local directories
            lines += """.. toctree::\n   :maxdepth: 1\n   :caption: """ + "Directories:\n\n"
            for dir_path in sorted(dir_paths):
                dirName = os.path.basename(os.path.normpath(dir_path))
                lines += "   " + dirName + "/index\n"

        if self.newFiles:
            with open(os.path.join(index_path, "index.rst"), "w") as f:
                f.write(lines)

    def generateAutoDoc(self, path, files_paths):
        if "/" in path:
            name = os.path.basename(path)
        sources = {}

        # Sort the files by language
        py_file_paths = sorted([s for s in files_paths if ".py" in s])
        c_file_paths = sorted([s for s in files_paths if ".c" in s or ".cpp" in s or ".h" in s or ".hpp" in s])
        rust_manifest_paths = sorted([s for s in files_paths if os.path.basename(s) == "Cargo.toml"])

        # Create the .rst file for C-Modules

        # Identify .h file and .c/.cpp that share the same basename
        c_file_basenames = []
        c_file_local_paths = []
        for c_file_path in c_file_paths:
            c_file_name = os.path.basename(c_file_path)
            c_file_local_paths.append(c_file_name)
            c_file_name = c_file_name[:c_file_name.rfind('.')]
            c_file_basenames.append(c_file_name)

        c_file_basenames = np.unique(c_file_basenames)

        lines = ""
        lines += ".. _" + name + ":\n\n"
        lines += name + "\n" + "=" * len(name) + "\n\n"
        lines += """.. toctree::\n   :maxdepth: 1\n   :caption: """ + name + ":\n\n"

        if not c_file_paths == []:
            # Identify where the module lives relative to source
            src_path = os.path.dirname(c_file_path)
            module_files = []
            sources = {}
            for c_file_basename in c_file_basenames:

                module_files_temp = []
                lines = ""
                if c_file_basename == 'orbitalMotion' or c_file_basename == 'rigidBodyKinematics':
                    pathToFolder = src_path.split("/" + c_file_basename)[0]
                    lines += ".. _" + c_file_basename + pathToFolder.split("/")[-1] + ":\n\n"
                else:
                    lines += ".. _" + c_file_basename + ":\n\n"

                # Link the path with the modules for Breathe
                # make sure the list of files match the base name perfectly
                # this avoids issues where one file name is contained in another
                # file name
                c_file_list_coarse = [s for s in c_file_local_paths if c_file_basename in s]
                c_file_list = [file_name for file_name in c_file_list_coarse if file_name.rsplit(".", 1)[0] == c_file_basename]
                module_files.extend(c_file_list)
                module_files_temp.extend(c_file_list)
                module_type = self.getModuleType(c_file_list)
                is_bsk_module = self.isBskCppOrCModule(src_path, c_file_basename)

                if is_bsk_module:
                    lines += self.writeModuleTitle("Module: " + c_file_basename, module_type)
                elif "architecture" in src_path or "utilities" in src_path:
                    lines += self.writeModuleTitle(c_file_basename)
                else:
                    lines += self.writeModuleTitle(c_file_basename)

                # pull in the module documentation file if it exists
                docFileName = os.path.join(src_path, c_file_basename + '.rst')
                if os.path.isfile(docFileName):
                    with open(docFileName, 'r', encoding="utf8") as docFile:
                        docContents = docFile.read()
                    lines += docContents + "\n\n"
                    lines += "----\n\n"

                # Populate the module's .rst
                for module_file in module_files_temp:
                    if ".h" in module_file:
                        if name == "_GeneralModuleFiles":
                            name += str(self.counter)
                            self.counter += 1
                        lines += (
                            f".. autodoxygenfile:: {module_file}\n"
                            f"   :project: {name}"
                        )
                        if module_file == "bsk_rust_module.h":
                            lines += (
                                "\n   :sections: innerclass briefdescription "
                                "detaileddescription public-attrib define"
                            )
                        lines += "\n\n"
                        # lines += """.. inheritance-diagram:: """ + module_file + """\n\n"""

                if self.newFiles:
                    with open(os.path.join(path,  c_file_basename + ".rst"), "w") as f:
                        f.write(lines)

            sources.update({name: (src_path, module_files)})

        # Create the .rst file for the python module
        if not py_file_paths == []:
            # Add the module path to sys.path so sphinx can produce docs
            src_dir = path[path.find("/")+1:]
            src_dir = src_dir[src_dir.find("/")+1:]
            sys.path.append(os.path.abspath(officialSrc+"/"+src_dir))

        for py_file in sorted(py_file_paths):
            fileName = os.path.basename(py_file)
            if fileName not in ["__init__.py"]:
                fileName = fileName[:fileName.rfind('.')]
                lines = ".. _"+ fileName + ":\n\n"
                if self.isBskPythonModule(py_file):
                    lines += self.writeModuleTitle("Module: " + fileName, "Python")
                else:
                    lines += self.writeModuleTitle(fileName)

                docFileName = os.path.join(os.path.dirname(py_file), fileName + '.rst')
                if os.path.isfile(docFileName):
                    with open(docFileName, 'r', encoding="utf8") as docFile:
                        docContents = docFile.read()
                    lines += docContents + "\n\n"
                    lines += "----\n\n"

                lines += """.. toctree::\n   :maxdepth: 1\n   :caption: """ + "Files" + ":\n\n"
                lines += """.. automodule:: """ + fileName + """\n   :members:\n   :show-inheritance:\n\n"""
                if self.newFiles:
                    with open(path+"/"+fileName+".rst", "w") as f:
                        f.write(lines)

        # Create the .rst file for Rust modules
        for manifest_file in rust_manifest_paths:
            if not self.isBskRustModule(manifest_file):
                continue

            src_path = os.path.dirname(manifest_file)
            module_name = os.path.basename(src_path)
            lines = ".. _" + module_name + ":\n\n"
            lines += self.writeModuleTitle("Module: " + module_name, "Rust")

            doc_file_name = os.path.join(src_path, module_name + ".rst")
            with open(doc_file_name, 'r', encoding="utf8") as doc_file:
                lines += doc_file.read() + "\n\n"

            generated_header = self._generated_rust_header(module_name)
            if generated_header:
                # Breathe caches AutoDoxygen projects by source directory.
                # Use one project for all generated Rust headers because they
                # share dist3/rust_headers; separate projects can otherwise
                # reuse another module's Doxygen input list.
                project_name = "BasiliskRustModules"
                lines += "Generated Module API\n--------------------\n\n"
                lines += (
                    "This C-compatible interface is generated from the Rust "
                    "module source.\n\n"
                )
                lines += (
                    ".. autodoxygenfile:: " + generated_header.name + "\n"
                    "   :project: " + project_name + "\n"
                    "   :sections: innerclass briefdescription "
                    "detaileddescription public-attrib public-func\n\n"
                )
                generated_headers = sorted(
                    path.name
                    for path in generated_header.parent.glob("*.h")
                    if path.is_file()
                )
                sources[project_name] = (
                    str(generated_header.parent),
                    generated_headers,
                )

            if self.newFiles:
                with open(os.path.join(path, module_name + ".rst"), "w") as f:
                    f.write(lines)

        return sources




    def run(self, srcDir):
        # Find all files and directories in the src directory
        paths_in_dir = self.grabRelevantFiles(srcDir)

        # In the local folder, divvy up files and directories
        file_paths, dir_paths = self.seperateFilesAndDirs(paths_in_dir)
        dir_paths = [
            directory
            for directory in dir_paths
            if not self._isCargoSourceDirectory(directory, file_paths)
            and not self._isCargoTargetDirectory(directory, file_paths)
        ]

        index_path = os.path.relpath(srcDir, officialSrc)
        if index_path == ".":
            index_path = "/"

        try:
            os.makedirs(officialDoc + index_path)
        except:
            pass

        # Populate the index.rst file of the local directory
        self.populateDocIndex(officialDoc+"/"+index_path, file_paths, dir_paths)

        # Generate the correct auto-doc function for C, C++, and python modules
        sources = self.generateAutoDoc(officialDoc+"/"+index_path, file_paths)

        # Need to update the translation layer from doxygen to sphinx (breathe)
        self.breathe_projects_source.update(sources)

        # Recursively go through all directories in source, documenting what is available.
        for dir_path in sorted(dir_paths):
            self.run(dir_path)

        if self.newFiles:
            return self.breathe_projects_source
        else:
            return

rebuild = not single_page_docname
officialSrc = "../../src"
officialDoc = "./Documentation/"

default_rust_header_dir = (
    Path(__file__).resolve().parents[2] / "dist3" / "rust_headers"
)
rust_header_dir = os.environ.get("BSK_RUST_HEADER_DIR", default_rust_header_dir)
fileCrawler = fileCrawler(rebuild, rust_header_dir)
import pickle

if rebuild:
    if os.path.exists(officialDoc):
        shutil.rmtree(officialDoc)
    # adjust the fileCrawler path to a local folder to just build a sub-system
    breathe_projects_source = fileCrawler.run(officialSrc)
    # breathe_projects_source = fileCrawler.run(officialSrc+"/fswAlgorithms")
    # breathe_projects_source = fileCrawler.run(officialSrc+"/simulation/environment")
    # breathe_projects_source = fileCrawler.run(officialSrc+"/moduleTemplates")
    # breathe_projects_source = fileCrawler.run(officialSrc+"/simulation/vizard")
    # breathe_projects_source = fileCrawler.run(officialSrc+"/architecture")
    breathe_projects_source = fileCrawler.run("../../examples")
    # breathe_projects_source = fileCrawler.run("../../supportData")
    # breathe_projects_source = fileCrawler.run("../../externalTools")
    with open("breathe.data", 'wb') as f:
        pickle.dump(breathe_projects_source, f)
else:
    if os.path.exists('breathe.data'):
        with open('breathe.data', 'rb') as f:
            breathe_projects_source = pickle.load(f)
    else:
        breathe_projects_source = {}
    #breathe_projects_source = pickle.load('breathe.data')
    #fileCrawler.run("../../../Basilisk/src/")

#TODO: Pickle the breathe_project_source and load that back in

# Example of how to link C with Breathe
# breathe_projects_source = {"BasiliskFSW": ("../../src/fswAlgorithms/attControl/mrpFeedback", ['mrpFeedback.c', 'mrpFeedback.h'])}
