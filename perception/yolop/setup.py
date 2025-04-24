from setuptools import setup, find_packages

setup(
    name='yolop',
    version='0.1',
    packages=find_packages(where='YOLOP'),
    package_dir={'': 'YOLOP'},
    install_requires=['setuptools'],
    author='HUST Vision and Learning Lab',
    description='YOLOP: You Only Look Once for Panoptic driving perception',
    url='https://github.com/hustvl/YOLOP',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
    ],
)
