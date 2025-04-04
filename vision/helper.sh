#!/bin/bash

# Function to create a virtual environment if it doesn't exist
create_venv_if_not_exists() {
    local VENV_DIR=".venv"

    # Check if the virtual environment directory exists
    if [ ! -d "$VENV_DIR" ]; then
        echo "Virtual environment not found. Creating a new one..."
        
        # Create the virtual environment
        python3 -m venv "$VENV_DIR"
        
        if [ $? -ne 0 ]; then
            echo "Error: Failed to create virtual environment."
            return 1
        fi
        
        echo "Virtual environment created successfully."
    else
        echo "Virtual environment already exists."
    fi
}

# Function to install dependencies from requirements.txt
install_requirements() {
    local VENV_DIR=".venv"

    # Activate the virtual environment
    source "$VENV_DIR/bin/activate"
    
    # Check if requirements.txt exists
    if [ -f "requirements.txt" ]; then
        echo "Installing dependencies from requirements.txt..."
        
        # Install dependencies
        pip install -r requirements.txt
        
        if [ $? -ne 0 ]; then
            echo "Error: Failed to install dependencies."
            deactivate
            return 1
        fi

        echo "Dependencies installed successfully."
    else
        echo "requirements.txt file not found. Skipping dependency installation."
    fi

    # Deactivate virtual environment
    deactivate
}

# Function to initialize the virtual environment
init_venv() {
    local VENV_DIR=$1

    # Create the virtual environment if it doesn't exist
    create_venv_if_not_exists "$VENV_DIR"
    
    if [ $? -ne 0 ]; then
        echo "Failed to create virtual environment. Exiting."
        return 1
    fi

    # Install the requirements in the virtual environment
    install_requirements "$VENV_DIR"
    
    if [ $? -ne 0 ]; then
        echo "Failed to install dependencies. Exiting."
        return 1
    fi
}

# Function to run the auto_uav.py script inside the virtual environment
run() {
    local VENV_DIR=".venv"
    local SCRIPT="auto_uav.py"

    export PYTHONDONTWRITEBYTECODE=1

    init_venv $VENV_DIR

    # Check if the virtual environment exists
    if [ ! -d "$VENV_DIR" ]; then
        echo "Error: Virtual environment not found."
        return 1
    fi

    # Check if auto_uav.py exists
    if [ ! -f "$SCRIPT" ]; then
        echo "Error: $SCRIPT not found."
        return 1
    fi

    # Activate the virtual environment
    source "$VENV_DIR/bin/activate"

    # Fix depthai path
    export PYTHONPATH="$(pwd)/../depthai:$PYTHONPATH"

    
    # Run the auto_uav.py script
    echo "Running $SCRIPT inside virtual environment..."
    python -u "$SCRIPT"
    
    # Capture the exit code of the script
    local EXIT_CODE=$?

    # Deactivate the virtual environment
    deactivate

    # Return the exit code of the script
    return $EXIT_CODE
}

# Usage message to show how to use the functions
usage() {
    echo "Usage: source setup_venv.sh"
    echo "Then call the following functions:"
    echo " - init_venv           : Create/activate venv and install requirements."
    echo " - run                : Run auto_uav.py inside the virtual environment."
}

# If the script is executed directly, show usage instructions
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    usage
fi
