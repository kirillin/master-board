#!/bin/bash

if [ -x "$(command -v udevd)" ]; then
    unshare --mount udevd --daemon
    udevadm trigger --action=add
fi

if [ "$1" = "shell" ]; then
    exec /bin/bash
else
    . /opt/esp/esp-idf/export.sh
    
    exec "$@"
fi