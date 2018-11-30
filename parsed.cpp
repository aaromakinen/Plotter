
#include "parsed.h"

parsed::parsed() : x(0), y(0), power(0),area_x(300),area_y(300),pen_up_code(160),pen_down_code(90),command_index(0), down(false)
{

}


parsed::~parsed()
{
}

#ifdef UNIT
#include <fstream>
#include <iostream>
using namespace std;
#endif
#ifdef UNIT
string readfile(ifstream & in) {
	string str;
	if (!in.eof()) {
		getline(in, str);
	}
	else {
		in.close();
	}
	return str;

}
#else
std::string parsed::readUART() {
	char str[80];
	std::string send;
	unsigned int i;
	uint32_t len = USB_receive((uint8_t *)str, 79);
	for (i=0;i<len;i++){
		if (buf.length()!=0){
			send = buf;
			buf ="";
		}
		send.push_back(str[i]);
		if(str[i]=='\n'){
			i++;
			for(;i<len;i++){
				buf.push_back(str[i]);
			}
			break;
		}
	}
	Board_UARTPutSTR(send.c_str());
	return send;
}


#endif
bool parsed::has_key(std::string str, char key) {
	for (unsigned int i = 0; i < str.length(); i++) {
		if (str[i] == key) {
			return true;
		}
	}
	return false;
}
double parsed::get_number(std::string str, char c) {
	char temp[10] = "";
	int k = 0;
	double checker;
	unsigned int i;
	for (i = 0; i < str.length(); i++) {
		if (str[i] == c) {
			i++;
			if (c == 'M') {
				i = 3;
			}
			if (str[i] == '-') {
				temp[0] = '0';
				return strtod(temp, NULL);
			}

			while (str[i] != ' ' && i < str.length() && str[i] != '\n') {
				temp[k] = (str[i]);
				k++;
				i++;
			}
			checker = strtod(temp, NULL);
			if (checker < 0) {
				checker = 0;
			}
			return checker;
		}
	}
	return 0;
}
void parsed::parser() {
#ifdef UNIT
	ifstream in("g_code.txt");
#endif // UNIT
	char buf [50];
	double i;
	std::string str;
#ifdef UNIT
	str = readfile(in);
#else
	str = readUART();
#endif // UNIT
	command_index = 0;
	if (str[0] == 'G') {
		command = 'G';
		if (str[1] == '1') {
			command_index=1;
			if (has_key(str, 'X') && has_key(str, 'Y')) {
				i = get_number(str, 'X');

				x = i*100;
				i = get_number(str, 'Y');
				y = i*100;
			}
		}
		else if (str[1] == '2' && str[2] == '8') {
			command_index=28;
			x = 0;
			y = 0;
		}
	}
	if (str[0] == 'M') {
		command = 'M';
		if (str[1] == '1') {
			if (str[2] == '0') {
				sprintf(buf,"M10 XY %d %d 0.00 0.00 A0 B0 H0 S80 U%d D%d\n",area_x,area_y,pen_up_code,pen_down_code);
				USB_send((uint8_t *)buf,strlen(buf));
			}
			else if ( str[2] == '1'){
				sprintf(buf,"M11 %d %d %d %d\n",limitx_2->read(),limitx_1->read(),limity_1->read(),limity_2->read());
				USB_send((uint8_t *)buf,strlen(buf));
			}
			else if ( str[2] != '1'){


					if (stoi((str.substr(3))) == pen_down_code) {
						command_index=1;
						down = true;
					}

					if (stoi((str.substr(3))) == pen_up_code) {
						command_index=2;
						down = false;
					}
			}
		}
		else if (str[1] == '2') {
			if(str[2] == '8'){
				command_index=28;
			}
			else{
				i = get_number(str,'U');
				pen_up_code=i;
				i = get_number(str,'D');
				pen_down_code=i;

			}
		}
		else if (str[1] == '4') {
			command_index=4;
			i = get_number(str, 'M');
			power = i;
		}
		else if (str[1] == '5') {
			i = get_number(str,'H');
			area_y=i;
			i = get_number(str,'W');
			area_x=i;
			command_index=3;
		}
	}
	sprintf(buf,"x=%d y=%d pen=%d\n\r",x,y,down);
	USB_send((uint8_t *)"OK\n\r",5);
	Board_UARTPutSTR(buf);
#ifdef UNIT
	switch (a) {
	case 0:
		if (par.down == false && par.power == 0 && par.x == 0 && par.y == 0) {
			cout << "ok\n";
		}
		break;
	case 1:
		if (par.down == false && par.power == 0 && par.x == 0.5&& par.y == 101.5) {
			cout << "ok\n";
		}
		break;
	case 2:
		if (par.down == true && par.power == 0 && par.x == 0.5 && par.y == 101.5) {
			cout << "ok\n";
		}
		break;
	case 3:
		if (par.down == true && par.power == 0 && par.x == 149.5 && par.y == 101.5) {
			cout << "ok\n";
		}
		break;
	case 4:
		if (par.down == true && par.power == 0 && par.x == 149.5 && par.y == 1.41) {
			cout << "ok\n";
		}
		break;
	case 5:
		if (par.down == true && par.power == 137 && par.x == 149.5 && par.y == 1.41) {
			cout << "ok\n";
		}
		break;
	case 6:
		if (par.down == true && par.power == 137 && par.x == 0.5 && par.y == 1.41) {
			cout << "ok\n";
		}
		break;
	case 7:
		if (par.down == true && par.power == 0 && par.x == 0.5 && par.y == 1.41) {
			cout << "ok\n";
		}
		break;
	case 8:
		if (par.down == true && par.power == 0 && par.x == 0.0 && par.y == 101.5) {
			cout << "ok\n";
		}
		break;
	case 9:
		if (par.down == false && par.power == 0 && par.x == 0.0 && par.y == 101.5) {
			cout << "ok\n";
		}
		break;
	}
	a++;
#endif

}


int parsed::get(int i){
	switch (i){
	case 0:
		return area_x;
	case 1:
		return area_y;
	case 2:
		return pen_up_code;
	case 3:
		return pen_down_code;
	case 4:
		return command;
	case 5:
		return x;
	case 6:
		return y;
	case 7:
		return power;
	case 8:
		return command_index;
	default:
		return -1;
	}

}
