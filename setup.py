from setuptools import setup, find_packages

setup(
    name="robot_dynamics",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "torch",
        "numpy",
        "pandas",
        "matplotlib",
        "scikit-learn",
    ],
) 