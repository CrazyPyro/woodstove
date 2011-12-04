mytarget=atmega8

default: clean ece554.hex

flash: ece554.hex
	msg="If this doesnt work, remember to run via sudo."
	avrdude -c usbasp -p $(mytarget) -u -U flash:w:ece554.hex	
	echo $msg

ece554.o:
	avr-gcc -g -Os -mmcu=$(mytarget) --std=c99 -o ece554.o -c ece554.c
ece554.elf: ece554.o
	avr-gcc -g -mmcu=$(mytarget) -o ece554.elf ece554.o
ece554.hex: ece554.elf
	avr-objcopy -j .text -j .data -O ihex ece554.elf ece554.hex

clean:
	rm -f ece554.o
	rm -f ece554.elf
	rm -f ece554.hex

