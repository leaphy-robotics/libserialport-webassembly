cmake_minimum_required(VERSION 3.0)

project(libserialport)


set(SOURCES
    serialport.c
    timing.c
    libserialport_internal.h
)

add_library(serialport STATIC ${SOURCES})
