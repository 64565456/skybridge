#!/bin/bash

PROJECT_PATH=$(cd "$(dirname "$0")"; pwd)
WORKSPACE_PATH=$(cd "$(dirname "$0")"; cd ..; pwd)

TOOL_NAME=flash_download_tool_3.9.5
TOOL_PATH=$WORKSPACE_PATH/tools/$TOOL_NAME

cp -r $TOOL_PATH build
cd $PROJECT_PATH/build

rm -rf ${TOOL_NAME}/bin
mkdir -p ${TOOL_NAME}/bin
cp bootloader/bootloader.bin \
   partition_table/partition-table.bin \
   nvs_data_initial.bin \
   ota_data_initial.bin \
   skybridge.bin ${TOOL_NAME}/bin

zip -r $TOOL_NAME.zip ${TOOL_NAME}
rm -rf ${TOOL_NAME}
