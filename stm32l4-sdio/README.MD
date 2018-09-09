# **SDIO** library for the **STM32L476RG** MCU.


- Card initialization (SDSCv1, SDSCv2, SDHC, SDXC, partial MMC)
- Read: polling or DMA, single or multiple block
- Write: polling or DMA, single or multiple block


Ported from L1 series, still needs some optimizations and polish.


`main.c` contains code to test performance of read and write operations, pretty stupid, but it works.


> Note: should work on all STM32L4 MCUs but not the STM32L4+

