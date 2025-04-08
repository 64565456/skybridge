PROJECT_PATH=$(cd "$(dirname "$0")"; pwd)

if [ ! -n "$IDF_PATH" ]; then
    echo "IDF_PATH not found"
    exit 1
fi

python3 $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
        generate \
        $PROJECT_PATH/nvs_data_initial.csv \
        $PROJECT_PATH/build/nvs_data_initial.bin 0x4000
