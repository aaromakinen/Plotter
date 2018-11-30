
#include "move.h"


move::move() : x_currentcoor(0),y_currentcoor(0),max_x(0),max_y(0),area_x(30000),area_y(30000),
dir_x(0,28,false,true,false),dir_y(1,0,false,true,false)
{

}

int move::desired_move(int x, int y){
	x_newcoor = x;
	y_newcoor = y;
	if (x_newcoor>area_x-100){
		x_newcoor = area_x-100;
	}
	else if(x_newcoor == 0){
		x_newcoor +=100;
	}
	if (y_newcoor>area_y-100){			//reduce one coordinate for both ends to make sure motor don't hit limit switches
		y_newcoor = area_y-100;
	}
	else if(y_newcoor == 0){
		y_newcoor += 100;
	}

	x_newcoor = (x_newcoor * max_x) / (area_x-200);
	y_newcoor = (y_newcoor * max_y) / (area_y-200);			//get the steps ratio to coordinates





	delta.x = x_newcoor - x_currentcoor;
	delta.y = y_newcoor - y_currentcoor;



	if(delta.x >= 0){
		dir_x.write(true);
		dirx=true;
	}
	else{
		dir_x.write(false);
		dirx=false;
	}							//get the direction for the motors

	if(delta.y >= 0){
		dir_y.write(false);
		diry=false;
	}
	else{
		dir_y.write(true);
		diry=true;
	}


	if (delta.y == 0){
		if ( delta.x == 0 ){
			return no_move;
		}
		return only_x;
	}
	if (delta.x == 0){		  	//Determine if the y or x axle difference is bigger or if only axle should move
		return only_y;
	}
	if(abs(delta.y) - abs(delta.x) > 0){
		return bigger_y;
	}
	else {
		return bigger_x;
	}





}


void move::change_dir(int axle,bool dir){
	if (axle == x_motor){
		dir_x.write(dir);
	}
	if (axle == y_motor){
		dir_y.write(dir);
	}
}
int move::getdelta(char c){
	switch (c){
	case 'x':
		return delta.x;
	case 'y':
		return delta.y;
	default:
		return -1;
	}
}
void move::setcoords(){
	x_currentcoor += delta.x;
	y_currentcoor += delta.y;
}
void move::setmax(int x, int y){
	max_x = x;
	max_y = y;
}
void move::setarea(int x,int y){
	area_x = x;
	area_y = y;
}
bool move::getdir(char c){
	switch (c){
	case 'x':
		return dirx;
	case 'y':
		return diry;
	default:
		return -1;
	}
}
