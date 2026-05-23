from setuptools import setup, find_packages

package_name = 'multi_tb3_system'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hochschule Darmstadt Student',
    maintainer_email='student@h-da.de',
    description='Multi-robot TurtleBot3 Burger leader-follower convoy system',
    license='Apache-2.0',
    entry_points={},
)
