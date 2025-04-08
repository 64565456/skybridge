set(srcs "")
set(include_dirs "")

list(APPEND srcs "main.c" "board.c" "startup.c" "runloop.c")
list(APPEND include_dirs ".")

list(APPEND srcs "crc/crc16.c" "crc/crc32.c")
list(APPEND include_dirs "crc")

list(APPEND srcs "dfu/dfu_progress.c"
                 "dfu/dfu_transfer.c"
                 "dfu/dfu.c")
list(APPEND include_dirs "dfu")

list(APPEND srcs "commu/com_proto.c"
                 "commu/com_uart.c"
                 "commu/channel_ble.c"
                 "commu/channel_esp.c"
                 "commu/channel_forward.c")
list(APPEND include_dirs "commu")

list(APPEND srcs "bluetooth/ble_main.c"
                 "bluetooth/bt_dm.c"
                 "bluetooth/bt_main.c"
                 "bluetooth/bt_stack.c")
list(APPEND include_dirs "bluetooth")

list(APPEND srcs "bluetooth/service/svc_ctrl.c"
                 "bluetooth/service/svc_trace.c"
                 "bluetooth/service/svc_ota.c")
list(APPEND include_dirs "bluetooth/service")

list(APPEND srcs "bluetooth/profile/prf_ctrl.c"
                 "bluetooth/profile/prf_trace.c"
                 "bluetooth/profile/prf_ota.c")
list(APPEND include_dirs "bluetooth/profile")

idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "${include_dirs}")
