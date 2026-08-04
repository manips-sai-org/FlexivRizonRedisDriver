# Set CPU governor and freq
sudo cpupower -c 1-9 frequency-set -g performance
sudo cpupower -c 1-9 frequency-set -d 4200MHz

# Run your RT task pinned to CPU 7
sudo env LD_LIBRARY_PATH=$HOME/rdk_install/lib:$LD_LIBRARY_PATH \
taskset --cpu-list 6-9 \
chrt -rr 79 \
./build/flexiv_rizon4_redis_driver_with_gripper config_titania.xml


