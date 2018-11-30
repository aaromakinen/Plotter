

#include <string>
#include "board.h"
#include <cstring>
#include "user_vcom.h"
#include "DigitalIoPin.h"
#ifdef UNIT
#include "stdafx.h"
#endif
extern DigitalIoPin *limitx_1;
extern DigitalIoPin *limitx_2;
extern DigitalIoPin *limity_1;
extern DigitalIoPin *limity_2;
class parsed
{
public:
	parsed();
	~parsed();
	void parser();

	int get(int i);




private:

	bool has_key(std::string str, char key);
	double get_number(std::string str, char c);
	std::string readUART();
	int x, y,power,area_x,area_y,pen_up_code,pen_down_code,command_index;
	bool down;
	char command;
	std::string buf;

};

