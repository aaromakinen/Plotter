
#include "DigitalIoPin.h"
#include "chip.h"
DigitalIoPin::DigitalIoPin() {

}
DigitalIoPin::DigitalIoPin(int port, int pin, bool input, bool pullup,bool invert) {
	port_b = port;
	pin_b = pin;
	inv = invert;
	in=input;
	if (input) {
		if (pullup) {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_MODE_PULLUP | IOCON_DIGMODE_EN | IOCON_INV_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_MODE_PULLUP | IOCON_DIGMODE_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);

			}
		}

		else {
			if (invert) {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,
						(IOCON_DIGMODE_EN | IOCON_INV_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			} else {
				Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin, (IOCON_DIGMODE_EN));
				Chip_GPIO_SetPinDIRInput(LPC_GPIO, port, pin);
			}
		}
	} else {
		Chip_IOCON_PinMuxSet(LPC_IOCON, port, pin,IOCON_MODE_INACT | IOCON_DIGMODE_EN);
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, port, pin);
		Chip_GPIO_SetPinState(LPC_GPIO, port, pin, true);

	}
}
DigitalIoPin::~DigitalIoPin() {
// TODO Auto-generated destructor stub
}
bool DigitalIoPin::read() {
	bool state = Chip_GPIO_GetPinState(LPC_GPIO, port_b, pin_b);
	if (!in) {
		if (inv) {
			state = !state;
		}

	}
	return state;
}
void DigitalIoPin::write(bool value) {
	if (!in) {
			if (inv) {
				value = !value;
			}

		}
	Chip_GPIO_SetPinState(LPC_GPIO, port_b, pin_b, value);
}
