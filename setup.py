# Author: Toru Seo <seo.t.aa@m.titech.ac.jp>
# Copyright (c) 2023 Toru Seo
# License: MIT License

from setuptools import setup
import uxsim

DESCRIPTION = "UXsim: traffic flow simulator"
NAME = 'uxsim'
AUTHOR = 'Toru Seo'
AUTHOR_EMAIL = 'seo.t.aa@m.titech.ac.jp'
URL = 'https://github.com/toruseo/UXsim'
LICENSE = 'MIT License'
DOWNLOAD_URL = 'https://github.com/toruseo/UXsim'
VERSION = uxsim.__version__
PYTHON_REQUIRES = ">=3.9"

INSTALL_REQUIRES = [
    'numpy>=1.21.5',
    'matplotlib>=3.5.2',
    'pillow>=9.2.0',
    'tqdm>=4.64.1',
    'scipy>=1.9.1',
    'pandas>=1.4.4',
]

EXTRAS_REQUIRE = {
}

PACKAGES = [
    'uxsim'
]

CLASSIFIERS = [
    'Intended Audience :: Science/Research',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3',
    'Programming Language :: Python :: 3 :: Only',
    'Topic :: Scientific/Engineering',
]

long_description = """# UXsim: traffic flow simulator

UXsim is a free, open-source macroscopic and mesoscopic network traffic flow simulator developed in Python. It is suitable for simulating large-scale (e.g., city-scale) vehicular transportation. It computes dynamic traffic flow in a network by using traffic flow models commonly utilized by transportation research. This simulator is written by pure Python and allows users to flexibly customize it.

- Github repo: https://github.com/toruseo/UXsim
- Technical reference: https://toruseo.jp/UXsim/docs/index.html
- arXiv preprint: https://arxiv.org/abs/2309.17114
- Fundamental theories (Japanese textbook): https://www.coronasha.co.jp/np/isbn/9784339052794/

MIT License

Copyright (c) 2023 Toru Seo
"""

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