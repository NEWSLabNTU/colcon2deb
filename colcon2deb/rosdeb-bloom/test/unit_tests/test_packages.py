import os

from bloom.packages import get_package_data

from ..utils.common import AssertRaisesContext, in_temporary_directory, redirected_stdio, user

test_data_dir = os.path.join(os.path.dirname(__file__), 'test_packages_data')


@in_temporary_directory
def test_get_package_data_fails_on_uppercase():
    user('git init .')

    with AssertRaisesContext(SystemExit, "Invalid package names, aborting."):
        with redirected_stdio():
            get_package_data(directory=test_data_dir)
