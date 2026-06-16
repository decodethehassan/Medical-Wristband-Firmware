cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smartwatch_all_sensors)

target_sources(app PRIVATE
  src/main.c
  src/as6221_task.c
  src/lsm6dso_task.c
  src/max30101_task.c
  src/ads1113_task.c
  src/w25n01_task.c
  src/ble_log_service.c
  src/log_backend_ble.c
  src/algo_v0.c         # <-- ADDED
)