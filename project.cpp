#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "user_vcom.h"
#include <move.h>
#include "parsed.h"
#include "ITM_write.h"
#include <mutex>
#include "Fmutex.h"
#include "DigitalIoPin.h"
#include <cmath>
#include "enums.h"


move *mover;
parsed *par;
DigitalIoPin *motorX;
DigitalIoPin *motorY;
DigitalIoPin *sw1;
DigitalIoPin *sw2;
DigitalIoPin *sw3;
DigitalIoPin *sw4;
DigitalIoPin *limitx_1;
DigitalIoPin *limitx_2;
DigitalIoPin *limity_1;
DigitalIoPin *limity_2;
DigitalIoPin *laser;

volatile uint32_t RIT_count;
xSemaphoreHandle sbRIT;
xSemaphoreHandle initialize;
volatile static bool ini_x = false;
volatile static bool ini_y = false;
int axle;
bool other_axle;

extern "C" {
void RIT_IRQHandler(void)
{
	static bool state = true;

	// This used to check if a context switch is required
	portBASE_TYPE xHigherPriorityWoken = pdFALSE;
	// Tell timer that we have processed the interrupt.
	// Timer then removes the IRQ until next match occurs
	Chip_RIT_ClearIntStatus(LPC_RITIMER); // clear IRQ flag
	if(RIT_count > 0) {
		if(mover->getdir('x') == true){
			if( (axle==only_x || ( (axle == bigger_x) || (axle == bigger_y && other_axle))) && !limitx_1->read() ){
				motorX->write(state);
			}
		}
		else{
			if( (axle == only_x || ((axle == bigger_x) || (axle == bigger_y && other_axle))) && !limitx_2->read() ){
				motorX->write(state);
			}
		}

		if(mover->getdir('y') == false){
			if( (axle == only_y || ((axle == bigger_y) || (axle == bigger_x && other_axle))) && !limity_1->read() ){
				motorY->write(state);
			}
		}
		else{
			if( (axle == only_y ||((axle == bigger_y) || (axle == bigger_x && other_axle))) && !limity_2->read()){
				motorY->write(state);
			}
		}

		state = !(bool)state;
		RIT_count--;
	}
	else {
		Chip_RIT_Disable(LPC_RITIMER); // disable timer
		// Give semaphore and set context switch flag if a higher priority task was woken up
		xSemaphoreGiveFromISR(sbRIT, &xHigherPriorityWoken);
	}
	// End the ISR and (possibly) do a context switch
	portEND_SWITCHING_ISR(xHigherPriorityWoken);
}
}

void RIT_start(int count, int us)
{
	uint64_t cmp_value;
	// Determine approximate compare value based on clock rate and passed interval
	cmp_value = (uint64_t) Chip_Clock_GetSystemClockRate() * (uint64_t) us / 1000000;
	// disable timer during configuration
	Chip_RIT_Disable(LPC_RITIMER);
	RIT_count = count;
	// enable automatic clear on when compare value==timer value
	// this makes interrupts trigger periodically
	Chip_RIT_EnableCompClear(LPC_RITIMER);
	// reset the counter
	Chip_RIT_SetCounter(LPC_RITIMER, 0);
	Chip_RIT_SetCompareValue(LPC_RITIMER, cmp_value);
	// start counting
	Chip_RIT_Enable(LPC_RITIMER);
	// Enable the interrupt signal in NVIC (the interrupt controller)
	NVIC_EnableIRQ(RITIMER_IRQn);
	// wait for ISR to tell that we're done
	if(xSemaphoreTake(sbRIT, portMAX_DELAY) == pdTRUE) {
		// Disable the interrupt signal in NVIC (the interrupt controller)
		NVIC_DisableIRQ(RITIMER_IRQn);
	}
	else {
		// unexpected error
	}
}

void SCT_Init(void)
{
	LPC_SCT0->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
	LPC_SCT0->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock = 1 MHz
	LPC_SCT0->MATCHREL[0].L = 20000-1;
	LPC_SCT0->MATCHREL[1].L = 1500; // match 1 used for duty cycle
	LPC_SCT0->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT0->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCT0->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT0->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCT0->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT0
	LPC_SCT0->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT0
	LPC_SCT0->CTRL_L &= ~(1 << 2); // unhalt it by clearing bit 2 of CTRL reg

	LPC_SCT1->CONFIG |= (1 << 17); // two 16-bit timers, auto limit
	LPC_SCT1->CTRL_L |= (72-1) << 5; // set prescaler, SCTimer/PWM clock = 1 MHz
	LPC_SCT1->MATCHREL[0].L = 100-1;
	LPC_SCT1->MATCHREL[1].L = 0; // match 1 used for duty cycle
	LPC_SCT1->EVENT[0].STATE = 0xFFFFFFFF; // event 0 happens in all states
	LPC_SCT1->EVENT[0].CTRL = (1 << 12); // match 0 condition only
	LPC_SCT1->EVENT[1].STATE = 0xFFFFFFFF; // event 1 happens in all states
	LPC_SCT1->EVENT[1].CTRL = (1 << 0) | (1 << 12); // match 1 condition only
	LPC_SCT1->OUT[0].SET = (1 << 0); // event 0 will set SCTx_OUT1
	LPC_SCT1->OUT[0].CLR = (1 << 1); // event 1 will clear SCTx_OUT1
	LPC_SCT1->CTRL_L &= ~(1 << 2); // unhalt it by clearing bit 2 of CTRL reg
}

static void ini(void *pvParameters) {
	vTaskDelay(100);
	int cnt = 0;
	int back_cnt = 0;
	int i,max_x,max_y;
	bool temp = true;
	while(1){
		if(xSemaphoreTake(initialize, portMAX_DELAY) == pdTRUE){			//only start initializing when command is given
			mover->change_dir(x_motor,true);
			ini_x = true;
			while ((!sw1->read()) && (!sw2->read()) && (!sw3->read()) && (!sw4->read()) ){		//motor in x axle goes to limit switch
				motorX->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
			}
			if (sw1->read()){
				limitx_1 = sw1;
			}
			else if (sw2->read()){
				limitx_1 = sw2;			//assign the limit switch
			}
			else if (sw3->read()){
				limitx_1 = sw3;
			}
			else if (sw4->read()){
				limitx_1 = sw4;
			}
			temp = true;
			mover->change_dir(x_motor,false);
			while (sw1->read() || sw2->read() || sw3->read() || sw4->read() ){			//come back from hitting the limit and count how many steps that took
				motorX->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				back_cnt++;
			}
			//second switch
			temp = true;
			while (!sw1->read() && !sw2->read() && !sw3->read() && !sw4->read() ){		//count steps as motor goes to the other limit switch

				motorX->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				cnt++;
			}
			cnt = cnt /2;
			if (sw1->read()){
				limitx_2 = sw1;
			}
			else if (sw2->read()){			//assign the limit switch
				limitx_2 = sw2;
			}
			else if (sw3->read()){
				limitx_2 = sw3;
			}
			else if (sw4->read()){
				limitx_2 = sw4;
			}
			if (cnt %2 !=0){
				cnt++;
			}
			temp = true;
			mover->change_dir(x_motor,true);
			while (sw1->read() || sw2->read() || sw3->read() || sw4->read() ){		//come back from the limit
				motorX->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				back_cnt++;
			}
			for (i = 0;i<back_cnt/10;i++){		//!111111111111111111111
				motorX->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
			}
			back_cnt = back_cnt /2;
			max_x =cnt - back_cnt;
			ini_x = false;

			// y-axle
			ini_y = true;
			cnt = 0;
			back_cnt = 0;
			mover->change_dir(y_motor,false);
			temp = true;
			while (!sw1->read() && !sw2->read() && !sw3->read() && !sw4->read() ){			//motor in y axle goes to limit switch

				motorY->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
			}
			if (sw1->read()){
				limity_1 = sw1;
			}
			else if (sw2->read()){			//assign the limit switch
				limity_1 = sw2;
			}
			else if (sw3->read()){
				limity_1 = sw3;
			}
			else if (sw4->read()){
				limity_1 = sw4;
			}
			temp = true;
			mover->change_dir(y_motor,true);
			while (sw1->read() || sw2->read() || sw3->read() || sw4->read() ){
				motorY->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				back_cnt++;
			}

			//second switch
			temp = true;
			while (!sw1->read() && !sw2->read() && !sw3->read() && !sw4->read() ){

				motorY->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				cnt++;
			}
			if (sw1->read()){
				limity_2 = sw1;
			}
			else if (sw2->read()){				//assign the limit switch
				limity_2 = sw2;
			}
			else if (sw3->read()){
				limity_2 = sw3;
			}
			else if (sw4->read()){
				limity_2 = sw4;
			}
			cnt = cnt /2;
			if (cnt %2 !=0){
				cnt++;
			}
			temp = true;
			mover->change_dir(y_motor,false);
			while (sw1->read() || sw2->read() || sw3->read() || sw4->read() ){
				motorY->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
				back_cnt++;
			}
			for (i = 0;i<back_cnt/10;i++){
				motorY->write(temp);
				temp = !(bool)temp;
				vTaskDelay(1);
			}

			back_cnt = back_cnt /2;
			max_y =cnt -back_cnt ;
			ini_y = false;

		}
		mover->setmax(max_x,max_y);		//send max steps to mover object

	}
}

static void loop(void *pvParameters) {
	vTaskDelay(10);
	int base_speed = 175;
	int i,steps,error,delta_x,delta_y;
	while(1){
		par->parser();
		if (par->get(command)=='M'){
			switch (par->get(command_index)){
			case 28:
				xSemaphoreGive(initialize);			//initializing call
				break;
			case 1:

				LPC_SCT0->MATCHREL[1].L = par->get(pen_down_code)*3.92+1000;			//pen up
				vTaskDelay(75);
				break;
			case 2:
				LPC_SCT0->MATCHREL[1].L = par->get(pen_up_code)*3.92+1000;		//pen down
				vTaskDelay(75);
				break;
			case 3:
				mover->setarea(100*par->get(areax),100*par->get(areay));		//change area (mm)
				break;
			case 4:
				if (par->get(power) == 0){
					base_speed = 175;
					LPC_SCT1->MATCHREL[1].L = 0;		//laser power
					break;
				}
				else{
					base_speed = 500;
				}
				LPC_SCT1->MATCHREL[1].L = ((par->get(power)) * 99)/255;
				break;
			}
		}
		if(par->get(command) == 'G'){
			if (par->get(command_index) == 1){
				axle = mover->desired_move(par->get(x), par->get(y));		//moving the motors
				delta_x = mover->getdelta('x');
				delta_y = mover->getdelta('y');
				switch(axle){

				// Bresenham's algorithm
				case bigger_x:
					error = 2 * abs(delta_y) - abs(delta_x);
					for(i = 0; i < std::abs(delta_x); i++){
						other_axle = false;
						if(error > 0){
							other_axle = true;
							error = error  - (2 * std::abs(delta_x));		//more or equal movement in x axle
						}
						error = error + (2 * std::abs(delta_y));
						RIT_start(2, base_speed);
					}
					break;
				case bigger_y:
					error = 2 * abs(delta_x) - abs(delta_y);
					for(i = 0; i < std::abs(delta_y); i++){
						other_axle = false;
						if(error > 0){
							other_axle = true;
							error = error  - (2 * std::abs(delta_y));		//more movement in y axle
						}

						error = error + (2 * std::abs(delta_x));

						RIT_start(2, base_speed);
					}
					break;
				case only_x:

					RIT_start(std::abs(delta_x) * 2  , base_speed);			//only x axle moves
					break;

				case only_y:
					RIT_start(std::abs(delta_y) * 2, base_speed);			//only y axle moves
					break;
				}
				mover->setcoords();
			}
			else if (par->get(command_index) == 28){
				steps = mover->desired_move(0, 0);				//go to origon
				if (steps > 0){
					RIT_start(steps,base_speed);
				}
			}

		}
	}
}



int main(void) {

#if defined (__USE_LPCOPEN)
	// Read clock settings and update SystemCoreClock variable
	SystemCoreClockUpdate();
#if !defined(NO_BOARD_LIB)
	// Set up and initialize all required blocks and
	// functions related to the board hardware
	Board_Init();
	// Set the LED to the state of "On"
#endif
#endif

	// TODO: insert code here
	ITM_init();
	Chip_RIT_Init(LPC_RITIMER);
	NVIC_SetPriority( RITIMER_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY + 5 );
	sbRIT = xSemaphoreCreateBinary();
	initialize = xSemaphoreCreateBinary();
	Chip_SCT_Init(LPC_SCT0);
	Chip_SWM_MovablePortPinAssign(SWM_SCT0_OUT0_O,0,10);
	Chip_SCT_Init(LPC_SCT1);
	Chip_SWM_MovablePortPinAssign(SWM_SCT1_OUT0_O,0,12);
	SCT_Init();

	mover= new move;
	par = new parsed;
	motorX = new DigitalIoPin(0,27,false,false,false);
	motorY = new DigitalIoPin(0,24,false,false,false);


	sw1 = new DigitalIoPin(1,3,true,true,true);
	sw2 = new DigitalIoPin(0,0,true,true,true);
	sw3 = new DigitalIoPin(0,9,true,true,true);
	sw4 = new DigitalIoPin(0,29,true,true,true);




	xTaskCreate(ini, "ini",
			configMINIMAL_STACK_SIZE * 8, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);
	xTaskCreate(loop, "main",
			configMINIMAL_STACK_SIZE * 8, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);

	xTaskCreate(cdc_task, "CDC",
			configMINIMAL_STACK_SIZE * 8, NULL, (tskIDLE_PRIORITY + 1UL),
			(TaskHandle_t *) NULL);
	volatile static int i = 0 ;
	vTaskStartScheduler();
	// Force the counter to be placed into memory
	// Enter an infinite loop, just incrementing a counter
	while(1) {
		i++ ;
	}
	return 0 ;
}
