"""
Auto-install the Maya stub at pytest collection time.

This conftest is automatically discovered by pytest when running tests
from ``tests/unit_tests/maya/`` or any subdirectory.

When running outside mayapy, it installs the Maya stub so that modules
touching ``maya.cmds`` can be imported for unit testing.  When running
inside mayapy, it is a no-op.
"""

from tests.unit_tests.maya.maya_stub import _is_real_maya_present, install_maya_stub

_real_maya = _is_real_maya_present()
if not _real_maya:
    install_maya_stub(profile="headless")
