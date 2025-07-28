# Run Command
qemu-system-arm \
  -M stm32vldiscovery \
  -kernel firmware.elf \
  -semihosting \
  -serial stdio \
  -nographic
