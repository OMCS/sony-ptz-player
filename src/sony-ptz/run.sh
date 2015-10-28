#!/bin/bash
OS=$(uname)

if [ "$OS" == "Linux" ]; then
	CONFIG='sony-ptz-linux.cfg'
else
	CONFIG='sony-ptz.cfg'
fi

mkdir ./build || cd build/ && cmake .. && make && cd .. && time player $CONFIG
