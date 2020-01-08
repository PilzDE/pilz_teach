#!/bin/bash

python-coverage erase
for f in tst_*.py
do
  python-coverage run -a -m pytest $f
done
python-coverage report --include="*pilz_teach*"
python-coverage html --include="*pilz_teach*"
