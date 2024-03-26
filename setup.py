# Author: Toru Seo <seo.t.aa@m.titech.ac.jp>
# Copyright (c) 2023 Toru Seo
# License: MIT License

from setuptools import setup, find_packages
#import uxsim

DESCRIPTION = "UXsim: traffic flow simulator"
NAME = 'uxsim'
AUTHOR = 'Toru Seo'
AUTHOR_EMAIL = 'seo.t.aa@m.titech.ac.jp'
URL = 'https://github.com/toruseo/UXsim'
LICENSE = 'MIT License'
DOWNLOAD_URL = 'https://github.com/toruseo/UXsim'
VERSION = "1.0.9"
PYTHON_REQUIRES = ">=3.9"

INSTALL_REQUIRES = [
    'numpy>=1.21.5',
    'matplotlib>=3.5.2',
    'pillow>=9.2.0',
    'tqdm>=4.64.1',
    'scipy>=1.9.1',
    'pandas>=1.4.4',
    'PyQt5>=5.15.7'
]

EXTRAS_REQUIRE = {
}

PACKAGES = find_packages()

CLASSIFIERS = [
    'Intended Audience :: Science/Research',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3 :: Only',
    'Topic :: Scientific/Engineering',
]


with open('README.md', encoding='utf-8') as fp:
    long_description = fp.read()

setup(name=NAME,
      author=AUTHOR,
      author_email=AUTHOR_EMAIL,
      maintainer=AUTHOR,
      maintainer_email=AUTHOR_EMAIL,
      description=DESCRIPTION,
      long_description=long_description,
      long_description_content_type='text/markdown',
      license=LICENSE,
      url=URL,
      version=VERSION,
      download_url=DOWNLOAD_URL,
      python_requires=PYTHON_REQUIRES,
      install_requires=INSTALL_REQUIRES,
      extras_require=EXTRAS_REQUIRE,
      packages=PACKAGES,
      classifiers=CLASSIFIERS,
      include_package_data=True
    )