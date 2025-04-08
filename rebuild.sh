#!/bin/bash

curdir=$(cd "$(dirname "$0")"; pwd)
cd $curdir

if [ ! -n "$IDF_PATH" ]; then
    source $curdir/../esp-idf/export.sh
fi

idf.py fullclean
idf.py set-target esp32c3
idf.py build
bash gen_nvs.sh
bash pack.sh
