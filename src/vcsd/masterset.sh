#!/bin/bash

# Insert modules
depmod -a ec_master
depmod -a ec_generic
modprobe -a ec_master
modprobe -a ec_generic

# Start ethercat daemon
/etc/init.d/ethercat restart
ls -al /dev/EtherCAT0
