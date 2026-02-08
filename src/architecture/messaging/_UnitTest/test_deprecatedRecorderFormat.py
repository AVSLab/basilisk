#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
"""
This test checks that when running with ``--recorderPropertyRollback`` we recover the
legacy output format for payload recorder fields that are non-numeric (a list of
dictionaries). Also, this test will automatically fail when the deprecation
deadline is passed.
"""
# If you're from the future and this test is failing with ``BSKUrgentDeprecationWarning``,
# it's time to remove support for the legacy output format. To do so:
# -  Remove ``--recorderPropertyRollback`` as build argument from the ``conanfile.py``,
#     and remove code saving ``RECORDER_PROPERTY_ROLLBACK`` in the cache_variables.
# -  In ``src/architecture/messaging/CMakeLists.txt``, remove the use of
#     ``RECORDER_PROPERTY_ROLLBACK``.
# -  In ``src/architecture/messaging/msgAutoSource/generateSWIGModules.py``, remove
#     the use of ``recorderPropertyRollback`` (modify any existing code that expects
#     this variable to be as if this variable were always ``False``).
# -  In ``src/architecture/messaging/newMessaging.ih``, remove ``explore_and_find_subattr``,
#     ``simple_attribute_map``, and ``__getattr__`` from ``%extend Recorder``. Also in
#     this file, from ``%extend std::vector<messageType ## Payload``, delete ``get_all``.
# -  Remove this test.
# -  In the list of known issues, remove mention of the use of ``--recorderPropertyRollback``
#     if no longer applicable for the version.
# -  Remove use of ``--recorderPropertyRollback`` from ``.github/workflows/pull-request.yml``.

from Basilisk.architecture import messaging
from Basilisk.utilities import deprecated

def test_deprecatedRecorderFormat():
    """test script to ensure any BSK include has access to messaging"""

    # if it's a property, then we compiled without --recorderPropertyRollback
    if isinstance(getattr(messaging.AccDataMsgRecorder, "accPkts", None), property):
        return

    msg = messaging.AccDataMsg()
    msg.write(msg.zeroMsgPayload)
    recorder = msg.recorder()

    recorder.Reset(0)
    recorder.UpdateState(0)

    # ignore the deprecation warning, but trigger it anyway
    # when the deprecation time limit expires, this will turn into a
    # non-ignored error
    with deprecated.ignore(r"Basilisk\.architecture\.messaging\..*Payload\..*Recorder\.__getattr__"):
        result = recorder.accPkts
        assert isinstance(result[0], dict) # expected, deprecated format

if __name__ == "__main__":
    test_deprecatedRecorderFormat()
