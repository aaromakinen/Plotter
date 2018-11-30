/*
 * move.h
 *
 *  Created on: Oct 2, 2018
 *      Author: ASUS
 */

#include "DigitalIoPin.h"
#include <stdlib.h>
#include "chip.h"
#include "enums.h"
#ifndef MOVE_H_
#define MOVE_H_
struct IntPoint {
	int x;
	int y;
};

class move{

public:
	move();
	bool direction();
	int desired_move(int x, int y);
	void initialize();
	void change_dir(int axle,bool dir);
	int getdelta(char c);
	bool getdir(char c);
	void setcoords();
	void setarea(int x,int y);
	void setmax(int x,int y);

private:
	int x_currentcoor,y_currentcoor,max_x,max_y,x_newcoor,y_newcoor,area_x,area_y;
	bool dirx,diry;
	IntPoint delta;
	DigitalIoPin dir_x;
	DigitalIoPin dir_y;


};





#endif /* MOVE_H_ */
