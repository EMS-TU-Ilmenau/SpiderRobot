from setuptools import setup, find_packages

setup(
	name='spiderrobot',
	version='0.0.6',
	packages=find_packages(),
	install_requires=['pyserial>=3.4']
)