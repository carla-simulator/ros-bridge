#!/bin/bash
autopep8 src/*.py --in-place --max-line-length=100
pylint --rcfile=../.pylintrc src/
