sudo cpufreq-set -c 4 -g performance
sudo cpupower -c 4 frequency-set -d 4200MHz 
sudo cpufreq-set -c 5 -g performance
sudo cpupower -c 5 frequency-set -d 4200MHz 
# sudo taskset --cpu-list 5 chrt -rr 80 ./build/flexiv_rizon4_redis_driver_with_gripper config_hero.xml
sudo taskset --cpu-list 5 chrt -rr 80 ./build/flexiv_rizon4_redis_driver_robot_only config_claudio.xml
# ./build/flexiv_rizon4_redis_driver_with_gripper config_hero.xml
