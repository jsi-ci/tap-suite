from setuptools import setup, find_packages

with open("README.md", "r") as readme_file:
    readme = readme_file.read()

requirements = ["numpy>=1.16"]

setup(
    name="create_clothoids",
    version="0.0.10",
    author="Erik Dovgan",
    author_email="erik.dovgan@ijs.si",
    description="A package to create multiple sequential asymmetric clothoid spline from points",
    long_description=readme,
    long_description_content_type="text/markdown",
    url="https://repo.ijs.si/erikdovgan/clothoids/",
    packages=find_packages(),
    install_requires=requirements,
    classifiers=[
        "Programming Language :: Python :: 3.6",
        "License :: MIT Licence",
    ],
)