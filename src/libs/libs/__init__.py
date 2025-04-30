import sys
import os

# Add all libraries inside 'libs' to the Python path
package_root = os.path.abspath(os.path.dirname(__file__))
libs_path = os.path.join(package_root, "..", "libs")

sys.path.insert(0, libs_path)

# Import submodules for easy access
from DeepGlobalRegistration import *