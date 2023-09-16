# Software Installation
All software has been tested on Ubuntu 20.04 and 22.04. To be able to run all simulation based software, install Drake (Version: 1.18.0) first into 
a python virtual 
environment, as described on the [Drake website](https://drake.mit.edu/pip.html#stable-releases). 

Here are the steps:
Clone the repository
```
git clone https://github.com/dfki-ric-underactuated-lab/ricmonk.git
```
It is recommended to create a virtual environment as the following 
```
cd ricmonk
python3 -m venv ricmonk_venv
ricmonk_venv/bin/pip install --upgrade pip
source ricmonk_venv/bin/activate
```
Naviagate to the directory where dependency text file exists:
```
cd software/python/simulation
```
Install the required dependencies:
```
pip install drake==1.18.0
pip install -r requirements.txt
```


