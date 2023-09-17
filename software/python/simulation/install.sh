#! /bin/bash
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Setup a virtual environment named: ricmonk_venv"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
python3 -m venv ricmonk_venv
ricmonk_venv/bin/pip install --upgrade pip
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
echo "Installing the dependencies:"
echo "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
ricmonk_venv/bin/pip install -r requirements.txt
