sudo cpufreq-set -c 1 -g performance
sudo cpufreq-set -c 2 -g performance
sudo cpufreq-set -c 3 -g performance
sudo cpufreq-set -c 4 -g performance
taskset --cpu-list 3 ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
