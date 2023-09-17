# Software Installation
All software has been tested on Ubuntu 20.04 and 22.04. To be able to run all simulation based software, install Drake first into 
a python virtual 
environment, as described on the [Drake website](https://drake.mit.edu/pip.html#stable-releases). 

Clone the repository
```
git clone https://github.com/dfki-ric-underactuated-lab/ricmonk.git
```

## Steps

Install virtual environment in your machine: 
```
sudo apt install python3-venv -y
```
It is recommended to create a virtual environment. Execute the `install.sh` bash script that exists in `ricmonk/software/python/simulation` directory. This setup the virtual environement and install the dependencies.
```
./install.sh
```

Activat the virtual environment. 
```
source ricmonk_venv/bin/activate
```

Navigate to the folders and execute the scripts.
