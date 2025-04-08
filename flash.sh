$IDF_PYTHON_ENV_PATH/bin/python $IDF_PATH/components/esptool_py/esptool/esptool.py \
-p /dev/ttyACM0 \
-b 921600 \
--before default_reset \
--after hard_reset \
--chip esp32c3 \
write_flash \
--flash_mode dio \
--flash_size 4MB \
--flash_freq 80m \
0x0 build/bootloader/bootloader.bin \
0x8000 build/partition_table/partition-table.bin \
0x9000 build/nvs_data_initial.bin \
0xd000 build/ota_data_initial.bin \
0x10000 build/skybridge.bin
