CROSS_COMPILE=arm-none-eabi
CXX=$(CROSS_COMPILE)-g++
AS=$(CROSS_COMPILE)-as
OBJCOPY=$(CROSS_COMPILE)-objcopy
CXXFLAGS=-mcpu=cortex-m3 -mthumb -ffreestanding -fno-exceptions -fno-rtti -nostdlib -nostartfiles
ASFLAGS = -mcpu=cortex-m3 -mthumb
LDFLAGS=-T linker.ld

all: firmware.bin

firmware.elf: startup.o main.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.s
	$(AS) $(ASFLAGS) $< -o $@

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -f *.o *.elf *.bin
