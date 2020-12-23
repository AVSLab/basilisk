# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
ifeq ($(OS),Windows_NT)
    # windows specific code
    SPHINXBUILD = py -3 -m sphinx
else
    # unix/macOS specific code
    SPHINXBUILD = python3 -m sphinx
endif
SPHINXOPTS    =
SOURCEDIR     = source
BUILDDIR      = build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

clean :
	-rm -rf $(BUILDDIR)
	-rm -rf $(SOURCEDIR)/Documentation
	-rm -rf $(SOURCEDIR)/breathe.data
	-rm -rf $(SOURCEDIR)/externalTools
	-rm -rf $(SOURCEDIR)/examples

view:
	    # works on macOS
		open build/html/index.html
	
.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)