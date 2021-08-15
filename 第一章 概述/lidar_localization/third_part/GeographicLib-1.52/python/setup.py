# setup.py, config file for distutils
#
# To install this package, execute
#
#   python setup.py install
#
# in this directory.  To run the unit tests, execute
#
#   python setup.py test
#
# To update the HTML page for this version, run
#
#   python setup.py register
#
# To upload the latest version to the python repository, run
#
#   python setup.py sdist --formats gztar,zip upload
#
# The initial version of this file was provided by
# Andrew MacIntyre <Andrew.MacIntyre@acma.gov.au>.

import setuptools

name = "geographiclib"
version = "1.52"

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
  name = name,
  version = version,
  author = "Charles Karney",
  author_email = "charles@karney.com",
  description = "The geodesic routines from GeographicLib",
  long_description = long_description,
  long_description_content_type = "text/markdown",
  url = "https://geographiclib.sourceforge.io/" + version + "/python",
  include_package_data = True,
  packages = setuptools.find_packages(),
  license = "MIT",
  keywords = "gis geographical earth distance geodesic",
  classifiers = [
    "Development Status :: 5 - Production/Stable",
    "Intended Audience :: Developers",
    "Intended Audience :: Science/Research",
    "License :: OSI Approved :: MIT License",
    "Operating System :: OS Independent",
    "Programming Language :: Python",
    "Topic :: Scientific/Engineering :: GIS",
    "Topic :: Software Development :: Libraries :: Python Modules",
  ],
  test_suite = "geographiclib.test.test_geodesic",
)
