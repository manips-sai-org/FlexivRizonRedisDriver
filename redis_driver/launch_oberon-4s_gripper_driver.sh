sudo cpufreq-set -c 1 -g performance
sudo cpupower -c 1 frequency-set -d 4200MHz 
sudo cpufreq-set -c 2 -g performance
sudo cpupower -c 2 frequency-set -d 4200MHz 
sudo cpufreq-set -c 3 -g performance
sudo cpupower -c 3 frequency-set -d 4200MHz 
sudo cpufreq-set -c 4 -g performance
sudo cpupower -c 4 frequency-set -d 4200MHz 
sudo taskset --cpu-list 4 chrt -rr 80 ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
# ./build/flexiv_rizon4_redis_driver_with_gripper config_oberon.xml
