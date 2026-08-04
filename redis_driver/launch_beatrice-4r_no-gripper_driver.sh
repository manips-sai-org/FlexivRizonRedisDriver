cpufreq-set -c 2 -g performance
cpupower -c 2 frequency-set -d 4200MHz 
cpufreq-set -c 3 -g performance
cpupower -c 3 frequency-set -d 4200MHz 
## Flexiv LD Library Path - 
# export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HOME/rdk_install/lib" 
LD_LIBRARY_PATH=~/rdk_install/lib taskset --cpu-list 3 chrt -rr 86  ./build/flexiv_rizon4_redis_driver_robot_only config_beatrice.xml
