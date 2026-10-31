# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Check the NumPy C API required by the shared message bindings."""

from pathlib import Path
import re
import sysconfig

import numpy as np
from setuptools._distutils.ccompiler import new_compiler
from setuptools._distutils.sysconfig import customize_compiler


def test_message_bindings_support_numpy124(tmp_path):
    """Compile the real NumPy include block against the installed build headers.

    Newer headers must not silently raise the runtime requirement above NumPy
    1.24, even when the runtime executing this test is newer.
    """
    template = (Path(__file__).parents[1] / "newMessaging.ih").read_text(encoding="utf-8")
    numpy_blocks = [
        block for block in re.findall(r"%\{(.*?)%\}", template, flags=re.DOTALL)
        if "#include <numpy/arrayobject.h>" in block
    ]
    assert len(numpy_blocks) == 1
    source = tmp_path / "numpy_api_probe.cpp"
    source.write_text(
        numpy_blocks[0]
        + '\nstatic_assert(NPY_FEATURE_VERSION <= NPY_1_24_API_VERSION, '
        '"Message bindings must support the NumPy 1.24 runtime");\n',
        encoding="utf-8",
    )
    compiler = new_compiler()
    customize_compiler(compiler)
    compiler.compile(
        [str(source)],
        output_dir=str(tmp_path),
        include_dirs=[sysconfig.get_path("include"), np.get_include()],
        macros=[("Py_LIMITED_API", "0x03090000")],
    )
