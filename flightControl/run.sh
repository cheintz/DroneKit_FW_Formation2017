#!/bin/bash


outFile=$(date +%Y_%m_%d__%H_%M_%S_textlog.log)

echo $LOGPATH/$outFile
stdbuf -oL python flightProgram.py --connect /dev/ttyACM0 | tee -i $LOGPATH/$outFile
