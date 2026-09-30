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

"""Check retry-server shutdown without hiding failures during a test run."""

import errno
import socket
from types import SimpleNamespace

import conftest
import pytest

rerunfailures = pytest.importorskip("pytest_rerunfailures")


def test_listener_closed_before_server_starts():
    """Allow pytest shutdown to close the socket before the thread starts."""
    # Bypass the constructor so the server cannot start its daemon thread yet.
    server = object.__new__(rerunfailures.ServerStatusDB)
    with socket.socket() as server.sock:
        conftest.pytest_unconfigure(SimpleNamespace(failures_db=server))
        assert server.sock.fileno() == -1
        server.run_server()


@pytest.mark.parametrize(
    "error_code, close_listener, suppressed",
    [
        (errno.ECONNABORTED, True, True),
        (errno.EBADF, True, True),
        (errno.ENOTSOCK, True, True),
        (getattr(errno, "WSAENOTSOCK", errno.ENOTSOCK), True, True),
        (errno.ECONNABORTED, False, False),
        (errno.EBADF, False, False),
        (errno.ENOTSOCK, False, False),
        (errno.EMFILE, True, False),
    ],
)
def test_server_error_handling(monkeypatch, error_code, close_listener, suppressed):
    """Suppress closed-socket failures while preserving other server errors."""
    failure = OSError(error_code, "injected server failure")

    class FailingServer:
        def run_server(self):
            if close_listener:
                conftest.pytest_unconfigure(SimpleNamespace(failures_db=self))
            raise failure

        def run_connection(self, conn):
            pass

    monkeypatch.setattr(rerunfailures, "ServerStatusDB", FailingServer)
    conftest._patch_rerunfailures_socket_cleanup()

    server = FailingServer()
    with socket.socket() as server.sock:
        if suppressed:
            server.run_server()
        else:
            with pytest.raises(OSError) as raised:
                server.run_server()
            assert raised.value is failure
