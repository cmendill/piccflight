#!/bin/bash

#set date & time on flight computer
ssh root@picture date -s @`( date -u +"%s" )`

