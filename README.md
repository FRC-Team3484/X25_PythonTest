# X25_PythonTest
This is a test of our 2025 code converted to python, to check for performance issues and to determine if Python is viable for us to switch to.

## Installation
Clone this repository:
```bash
git clone https://github.com/FRC-Team3484/X25_PythonTest
cd X25_PythonTest
```

Create a virtual environment:
```bash
python -m venv .venv
```

Activate it:
```bash
source ./.venv/bin/activate
```

Install the dependencies:
```bash
pip install -r requirements.txt
```

## Usage
VSCode tasks are included for many of the below terminal commands. You can run them by opening the command prompt (`Ctrl+Shift+P`) and then run `Run Task > (command)`.

Sync additional dependencies found in `pyproject.toml`:
```bash
robotpy sync
```
You shouldn't have to do this, as we're using pip and a requirements file here. If a new dependency is added in `pyproject.toml`, however, this command should be run, and then the `requirements.py`

Deploy robot code on the robot:
```bash
robotpy deploy
```

Run code tests on your local machine. Use this to test potential issues with robot code before finishing your implementation
```bash
robotpy test
```