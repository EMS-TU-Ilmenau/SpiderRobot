from setuptools import setup, find_packages

setup(
	name='spiderrobot',
	version='0.1.0',
	packages=find_packages(),
	install_requires=['pyserial>=3.4']
)