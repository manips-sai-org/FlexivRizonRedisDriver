sudo cpufreq-set -c 2 -g performance
sudo cpupower -c 2 frequency-set -d 5000MHz 
sudo cpufreq-set -c 3 -g performance
sudo cpupower -c 3 frequency-set -d 5000MHz 
sudo cpufreq-set -c 4 -g performance
sudo cpupower -c 4 frequency-set -d 5000MHz 
LD_LIBRARY_PATH=~/rdk_install_flexiv/lib taskset --cpu-list 3 chrt -f 86 ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
# ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
