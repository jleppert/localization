# Localization, motor control and robotic drive control

## Versions

### OS/Hardware 
- NVidia Jetson Xavier NX
- Ubuntu 18.04 (stripped down image)
- Nvidia JetPack 4.6.1
- L4T (Linux4Tegra) 32.7.1 
- kernel with RT patches: 4.9.253-rt168-tegra (4.9.253-rt168-tegra #1 SMP PREEMPT RT Sat Feb 19 08:51:03 PST 2022 aarch64 aarch64 aarch64 GNU/Linux) 

### Toolchain
- GCC 9.4.0
- cmake 3.23.1
- node 14.18.2
- python 3.6.9 
- redis 6.2.6 (v=6.2.6 sha=00000000:0 malloc=jemalloc-5.1.0 bits=64 build=fbde9e8a8cf76ff6)
- eigen 3.4.0
- msgpack-c, msgpack-cpp (c3df1bb26ebdd01d618ecca7ae2d6b4e37d5abd7)
- redis-plus-plus (58084931ed1a056d91fe96da7b9ea81fa023560a)
- hiredis (eaa2a7ee77f4ce25e73a23e6030d4fa4d138cb11)
- allwpilib (fork)

## Flashing Tegra EMMC Modules

## Update bootloader:
- `./flash.sh -r -k CPUBL-CFG jetson-xavier-nx-devkit-emmc mmcblk0p1` 
