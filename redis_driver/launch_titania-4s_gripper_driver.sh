sudo cpufreq-set -c 1 -g performance
sudo cpufreq-set -c 2 -g performance
sudo cpufreq-set -c 3 -g performance
sudo cpufreq-set -c 4 -g performance
sudo cpufreq-set -c 5 -g performance
chrt -rr 99 ./build/flexiv_rizon4_redis_driver_with_gripper config_titania.xml
