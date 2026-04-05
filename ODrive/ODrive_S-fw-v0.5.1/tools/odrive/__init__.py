
import os
import sys
# Do not prepend ../Firmware/fibre/python: use the `fibre` package shipped with
# this install (tools/fibre). Prepending the firmware tree breaks editable
# installs and can shadow the bundled package when cwd is under Firmware/.

# Syntactic sugar to make usage more intuative.
# Try/pass used to break install-time dep issues
try:
    import fibre
    find_any = fibre.find_any
    find_all = fibre.find_all
except:
    pass

# Standard convention is to add a __version__ attribute to the package
from .version import get_version_str
__version__ = get_version_str()
del get_version_str
