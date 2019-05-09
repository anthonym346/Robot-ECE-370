from setuptools import setup, find_packages

with open("README.md", "r") as readme_file:
    readme = readme_file.read()

requirements = ["pip>=9.0.1", "wheel>=0.33.1", "readchar>=2.0.1", "pynpuy>=1.4.2"]

setup(
    name="Robot-ECE370",
    version="1.2.1",
    author="Anthony Matthews",
    author_email="amatthe9@gmu.edu",
    description="A package to control a robot using a feather m0, IMU, IR sensors, and DRV8833 motor driver",
    long_description=readme,
    long_description_content_type="text/markdown",
    url="https://github.com/anthonym346/Robot-ECE-370/",
    packages=find_packages(),
    install_requires=requirements,
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    ],
)
