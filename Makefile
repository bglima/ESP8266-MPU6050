PROGRAM=button

# Adjust the Makefile for your own computer!
# The format is: include PATH_TO_COMMON.MK
#
# Bruno's PC
# Remember to alter path:
LD_LIBRARY_PATH="$LD_LIBRARY_PATH":/home/brunolima/Projetos/SE/esp-open-sdk/xtensa-lx106-elf/bin
export LD_LIBRARY_PATH
include ../esp-open-rtos/common.mk
# 
# Felipe's PC 
#
