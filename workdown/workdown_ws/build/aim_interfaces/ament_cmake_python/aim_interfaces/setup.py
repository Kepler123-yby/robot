from setuptools import find_packages
from setuptools import setup

setup(
    name='aim_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('aim_interfaces', 'aim_interfaces.*')),
)
