#!/bin/bash

find . -name "*.c" -exec utilities/format-minimal.sh {} \;
find . -name "*.h" -exec utilities/format-minimal.sh {} \; 
