sudo cpufreq-set -c 1 -g "performance"
taskset 0x2 ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
