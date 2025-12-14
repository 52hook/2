from setuptools import find_packages
from setuptools import setup

setup(
    name='yhs_can_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('yhs_can_interfaces', 'yhs_can_interfaces.*')),
)
