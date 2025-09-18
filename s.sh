#!/bin/bash

echo "0: Install"
echo "1: Uninstall"
echo ">> $1"

if [ "$1" -eq 0 ]; then
    echo "Installing ..."
elif [ "$1" -eq 1 ]; then
    echo "Uninstalling ..."
else
    echo "Invalid option: $1"
    echo "Usage: $0 {0|1}"
    exit 1
fi
