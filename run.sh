#!/bin/bash
set -e
rm -rf out
javac -d out $(find src -name "*.java")
java -cp out dip.Main FinalDIP67.bmp --font=micrenc.ttf --debug
