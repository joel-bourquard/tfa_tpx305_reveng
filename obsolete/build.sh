#!/bin/bash
gcc -Wall -march=native -O3 -o cracker custom-crc-bruteforcer.c && ./cracker
