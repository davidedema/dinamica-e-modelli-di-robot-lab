#!/usr/bin/env bash

set -e  # Exit immediately if a command fails

ENV_NAME="dmr"

echo "Creating virtual environment: $ENV_NAME"
python3 -m venv $ENV_NAME

echo "Activating virtual environment"
source $ENV_NAME/bin/activate

echo "Upgrading pip"
pip install --upgrade pip

echo "Installing requirements"
pip install -r requirements.txt

echo "Installation complete."
echo "To deactivate the environment, run:"
echo "deactivate"
echo "To activate the environment later, run:"
echo "source $ENV_NAME/bin/activate"

echo "Checking connection to MyCobot..."
python3 check_connection.py