import pytest


def pytest_addoption(parser):
    parser.addoption("--show_plots", action="store", default="true",
        help="test shall display plots: true or false")


@pytest.fixture
def show_plots(request):
     return request.config.getoption("--show_plots")
