
CC=avr-gcc
PROGRAMMER=usbtiny
CC_FLAGS=-lm -Os
#PROGRAMMER=avrisp2
PORT=usb
CPU=attiny85

NAME=incubalibre
#NAME=test_adc
#NAME=test_timer1

SOURCE=${NAME}.c
OBJECT=${NAME}.o
ELF=${NAME}.elf
HEX=${NAME}.hex

LFUSE=0x42
HFUSE=0xDF
EFUSE=0xFF

object: ${SOURCE}
	${CC} -I${CC_FLAGS} -mmcu=${CPU} -c ${SOURCE}

elf: object
	${CC} -I${CC_FLAGS} -mmcu=${CPU} -o ${ELF} ${OBJECT}

hex: elf
	avr-objcopy -j .text -j .data -O ihex ${ELF} ${HEX}

avrdude: hex
	avrdude -c ${PROGRAMMER} -P ${PORT} -p ${CPU} -U flash:w:${HEX}:i

rfuse:
	avrdude -c ${PROGRAMMER} -p ${CPU} -U lfuse:r:-:h 2> /dev/null
	avrdude -c ${PROGRAMMER} -p ${CPU} -U hfuse:r:-:h 2> /dev/null
	avrdude -c ${PROGRAMMER} -p ${CPU} -U efuse:r:-:h 2> /dev/null

wfuse:
	avrdude -c ${PROGRAMMER} -p ${CPU} -U lfuse:w:${LFUSE}:m 2> /dev/null
	avrdude -c ${PROGRAMMER} -p ${CPU} -U hfuse:w:${HFUSE}:m 2> /dev/null
	avrdude -c ${PROGRAMMER} -p ${CPU} -U efuse:w:${EFUSE}:m 2> /dev/null


clean:
	rm -rf ${ELF} ${OBJECT} ${HEX}
