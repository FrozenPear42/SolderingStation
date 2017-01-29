target remote localhost:2331
monitor reset
load build/OLED.elf
monitor reset
monitor go
disconnect
quit