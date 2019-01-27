arm-none-eabi-gcc -I../include -static -mthumb -g -mcpu=cortex-m3 *.c -T linker_script.ld -o main.elf -nostartfiles 
arm-none-eabi-objcopy -g -O binary main.elf main.bin
