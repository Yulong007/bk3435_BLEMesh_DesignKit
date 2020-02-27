#!/bin/bash

find . -name "*.c" | xargs dos2unix
find . -name "*.h" | xargs dos2unix
