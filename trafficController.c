/* Traffic Light Controller
 *
 * --- Code is best viewed with the tab size of 4. ---
 */

#include <system.h>
#include <sys/alt_alarm.h>
#include <sys/alt_irq.h>
#include <altera_avalon_pio_regs.h>
#include <alt_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// A template for COMPSYS 303 Assignment 1

// NOTE: You do not need to use this! Feel free
// to write your own code from scratch if you
// want, this is purely an example

// FUNCTION PROTOTYPES
// Timer ISRs
alt_u32 tlc_timer_isr(void* context);
alt_u32 camera_timer_isr(void* context);

//  Misc
// Others maybe added eg LEDs / UART
void lcd_set_mode(unsigned int mode);

// TLC state machine functions
void init_tlc(void);
void simple_tlc(int* state);
void pedestrian_tlc(int* state);
void configurable_tlc(int* state);
int config_tlc(int *tl_state);
void camera_tlc(int* state);

// Button Inputs / Interrupts
void buttons_driver(int* button);
void handle_mode_button(unsigned int* taskid);
void handle_vehicle_button(void);
void init_buttons_pio(void);
void NSEW_ped_isr(void* context, alt_u32 id);
void do_led(char lightState);

// Red light Camera
void clear_vehicle_detected(void);
void vehicle_checked(void);
int is_vehicle_detected(void);
int is_vehicle_left(void);

// Configuration Functions
int update_timeout(void);
void config_isr(void* context, alt_u32 id);
void buffer_timeout(unsigned int value);
void timeout_data_handler(void);


// CONSTANTS
#define OPERATION_MODES 0x03	// number of operation modes (00 - 03 = 4 modes)
#define CAMERA_TIMEOUT	2000	// timeout period of red light camera (in ms)
#define TIMEOUT_NUM 6			// number of timeouts
#define TIME_LEN 8				// buffer length for time digits
#define ESC 27
#define CLEAR_LCD_STRING "[2J"


// USER DATA TYPES
// Timeout buffer structure
typedef struct  {
	int index;
	unsigned int timeout[TIMEOUT_NUM];
} TimeBuf;


// GLOBAL VARIABLES
static alt_alarm tlc_timer;		// alarm used for traffic light timing
static alt_alarm camera_timer;	// alarm used for camera timing

// NOTE:
// set contexts for ISRs to be volatile to avoid unwanted Compiler optimisation
static volatile int tlc_timer_event = 0;
static volatile int camera_timer_event = 0;
static volatile int pedestrianNS = 0;
static volatile int pedestrianEW = 0;

// 4 States of 'Detection':
// Car Absent
// Car Enters
// Car is Detected running a Red
// Car Leaves
static int vehicle_detected = 0;

// Traffic light timeouts
static unsigned int timeout[TIMEOUT_NUM] = {500, 6000, 2000, 500, 6000, 2000};
static TimeBuf timeout_buf = { -1, {500, 6000, 2000, 500, 6000, 2000} };

// UART
FILE* fp;

// Traffic light LED values
//static unsigned char traffic_lights[TIMEOUT_NUM] = {0x90, 0x50, 0x30, 0x90, 0x88, 0x84};
// NS RGY | EW RGY
// NR,NG | NY,ER,EG,EY
static unsigned char traffic_lights[TIMEOUT_NUM] = {0x24, 0x21, 0x22, 0x24, 0x0C, 0x14};

enum traffic_states {RR0, GR, YR, RR1, RG, RY};

static unsigned int mode = 0;
// Process states: use -1 as initialization state
static int proc_state[OPERATION_MODES + 1] = {-1, -1, -1, -1};

FILE *lcd;

// Initialize the traffic light controller
// for any / all modes
void init_tlc(void)
{
	
}
	
	
/* DESCRIPTION: Writes the mode to the LCD screen
 * PARAMETER:   mode - the current mode
 * RETURNS:     none
 */
void lcd_set_mode(unsigned int mode)
{
	fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
	fprintf(lcd, "Mode: %d\n", mode);
}

/* DESCRIPTION: Performs button-press detection and debouncing
 * PARAMETER:   button - referenced argument to indicate the state of the button
 * RETURNS:     none
 */
void buttons_driver(int* button)
{
	// Persistant state of 'buttons_driver'
	static int state = 0;
	
	*button = 0;	// no assumption is made on intial value of *button
	// Debounce state machine
		// call handle_mode_button()
}

/* DESCRIPTION: Simple traffic light controller
 * PARAMETER:   state - state of the controller
 * RETURNS:     none
 */
void simple_tlc(int* state)
{
	static int started = -1;
	void *context = (void*) &tlc_timer_event;
	int *trigger;

	if (*state == -1) {
		// Process initialization state
		init_tlc();
		(*state)++;
		return;
	}
	
	if(*state == 0 && started == -1) {
		//Start timer for state 0
		alt_alarm_start(&tlc_timer, timeout[*state], tlc_timer_isr, context);
		do_led(traffic_lights[*state]);
		started = 1;
		return;
	}

	trigger = (int*) context;

    while(1) {
        // If the timeout has occured
        if(*trigger == 1) {
        	//Increment state counter
        	 if(*state == 5) {
        		 //Go back to state 0 when cycle completed
        		 *state = 0;
            } else {
            	(*state)++;
        	}
            *trigger = 0;
            //Process state
            alt_alarm_start(&tlc_timer, timeout[*state], tlc_timer_isr, context);
            printf("Doing LED's for state: %d\n", *state);
            do_led(traffic_lights[*state]);

            return;
        }
    }
}

void do_led(char lightState) {
	IOWR_ALTERA_AVALON_PIO_DATA(LEDS_GREEN_BASE, lightState);
	return;
}


/* DESCRIPTION: Handles the traffic light timer interrupt
 * PARAMETER:   context - opaque reference to user data
 * RETURNS:     Number of 'ticks' until the next timer interrupt. A return value
 *              of zero stops the timer. 
 */
alt_u32 tlc_timer_isr(void* context)
{
	volatile int* trigger = (volatile int*)context;
	*trigger = 1;
	return 0;
}


/* DESCRIPTION: Initialize the interrupts for all buttons
 * PARAMETER:   none
 * RETURNS:     none
 */
void init_buttons_pio(void)
{
	// Initialize NS/EW pedestrian button		
    int buttonValue = 1;
    void *buttonIsrContext = (void*) &buttonValue;
	// Reset the edge capture register
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0);
    // Enable interrupts for key 0 and key 1
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(BUTTONS_BASE, 0x3);
    alt_irq_register(BUTTONS_IRQ, buttonIsrContext, NSEW_ped_isr);
	
}


/* DESCRIPTION: Pedestrian traffic light controller
 * PARAMETER:   state - state of the controller
 * RETURNS:     none
 */
void pedestrian_tlc(int* state)
{	
	void *context = (void*) &tlc_timer_event;
	int *trigger;
	static int started = -1;

	if (*state == -1) {
		// Process initialization state
		init_tlc();
		(*state)++;
		return;
	}

	if(*state == 0 && started == -1) {
		//Start timer for state 0
		alt_alarm_start(&tlc_timer, timeout[*state], tlc_timer_isr, context);
		do_led(traffic_lights[*state]);
		started = 1;
		return;
	}

	trigger = (int*) context;

    while(1) {
        // If the timeout has occured
        if(*trigger == 1) {
        	if(*state == 5) {
        		*state = 0;
        	} else {
        	    (*state)++;
        	}
            *trigger = 0;
            //Process state
            if(*state == 1 && pedestrianNS) {
                // Also turn on pedestrian NS light (Bit 6 of LEDS_GREEN)
//            	printf("State is 1 and pedestrian has queried\n");
                do_led(traffic_lights[*state] | 0x040);
                pedestrianNS = 0;
            } else if(*state == 4 && pedestrianEW) {
                // Also turn on pedestrian EW light (Bit 7 of LEDS_GREEN)
//            	printf("State is 4 and pedestrian has queried\n");
                do_led(traffic_lights[*state] | 0x080);
//                printf("LED: %d\n", traffic_lights[*state] | 0x80);
                pedestrianEW = 0;
            } else {
//            	printf("No pedestrians present\n");
            	do_led(traffic_lights[*state]);
            }
        	alt_alarm_start(&tlc_timer, timeout[*state], tlc_timer_isr, context);
            //Increment state counter

            return;
        }
    }
	
}


/* DESCRIPTION: Handles the NSEW pedestrian button interrupt
 * PARAMETER:   context - opaque reference to user data
 *              id - hardware interrupt number for the device
 * RETURNS:     none
 */
void NSEW_ped_isr(void* context, alt_u32 id)
{
    int temp;
    temp = IORD_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE);
    //Do bit reading instead of equality
    if(temp & 0x01) {
        pedestrianNS = 1;
    } else if(temp & 0x02) {
        pedestrianEW = 1;
    }
    //Clear edge Capture Register
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(BUTTONS_BASE, 0);
//    printf("Button: %i\n", temp);
	
	
}


/* DESCRIPTION: Configurable traffic light controller
 * PARAMETER:   state - state of the controller
 * RETURNS:     none
 */
/*
If there is new configuration data... Load it.
Else run pedestrian_tlc();
*/
void configurable_tlc(int* state)
{	
	if (*state == -1) {
		// Process initialization state
		return;
	}
	
	
}


/* DESCRIPTION: Implements the state machine of the traffic light controller in 
 *              the ***configuration*** phase
 * PARAMETER:   tl_state - state of the traffic light
 * RETURNS:     Returns the state of the configuration phase
 */
/*
Puts the TLC in a 'safe' state... then begins update
*/
int config_tlc(int* tl_state)
{
	// State of configuration
	static int state = 0;
	
	if (*tl_state == -1) {
		// Process initialization state
		state = 0;
		return 0;
	}
	
	return state;
}


/* DESCRIPTION: Parses the configuration string and updates the timeouts
 * PARAMETER:   none
 * RETURNS:     none
 */
/*
 buffer_timeout() must be used 'for atomic transfer to the main timeout buffer'
*/
void timeout_data_handler(void)
{
	
}


/* DESCRIPTION: Stores the new timeout values in a secondary buffer for atomic 
 *              transfer to the main timeout buffer at a later stage
 * PARAMETER:   value - value to store in the buffer
 * RETURNS:     none
 */
void buffer_timeout(unsigned int value)
{
	
}


/* DESCRIPTION: Implements the update operation of timeout values as a critical 
 *              section by ensuring that timeouts are fully received before 
 *              allowing the update
 * PARAMETER:   none
 * RETURNS:     1 if update is completed; 0 otherwise
 */
int update_timeout(void)
{
	
}

/* DESCRIPTION: Handles the red light camera timer interrupt
 * PARAMETER:   context - opaque reference to user data
 * RETURNS:     Number of 'ticks' until the next timer interrupt. A return value
 *              of zero stops the timer. 
 */
alt_u32 camera_timer_isr(void* context)
{
	volatile int* trigger = (volatile int*)context;
	*trigger = 1;
	return 0;
}	

/* DESCRIPTION: Camera traffic light controller
 * PARAMETER:   state - state of the controller
 * RETURNS:     none
 */
 /*
 Same functionality as configurable_tlc
 But also handles Red-light camera
 */
void camera_tlc(int* state)
{
	if (*state == -1) {
		configurable_tlc(state);
		return;
	}	
	
}


/* DESCRIPTION: Simulates the entry and exit of vehicles at the intersection
 * PARAMETER:   none
 * RETURNS:     none
 */
void handle_vehicle_button(void)
{
	
}

// set vehicle_detected to 'no vehicle' state
void clear_vehicle_detected(void) 
{  
}
// set vehicle_detected to 'checking' state
void vehicle_checked(void) 
{
}
// return true or false if a vehicle has been detected
int is_vehicle_detected(void) 
{
}
// return true or false if the vehicle has left the intersection yet
int is_vehicle_left(void) 
{
}





int main(void)
{	
	int switchState = 0;
	int i;
	char mask;
	int tempMode;
	// initialise lcd
	lcd = fopen(LCD_NAME, "w");
	lcd_set_mode(0);
	init_buttons_pio();			// initialize buttons
	mode = 0;
	while (1) {
//		printf("Hello world\n");
		// Button detection & debouncing
//		printf("Switch State: %i\n", switchState);
//		printf("Proc state: %d\n", proc_state[0]);
//		printf("\n");

		
		// if Mode button pushed and we are in a safe state (RR or Uninitialised)
		if(proc_state[mode] == 0 || proc_state[mode] == 3 || proc_state[mode] == -1) {
			switchState = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
			for(i = 0; i < 4; i++){
				mask = 1 << i;
				if(mask & switchState) {
					tempMode = i;
					if(tempMode != mode) {
						//Timer does not get REstarted when we go back to a tlc in mode -1
						proc_state[mode] = -1;
						alt_alarm_stop(&tlc_timer);
						mode = tempMode;
					}
					break;
				}
			}
		}
			// Set current TLC state to -1
			// handle_mode_button to change state & display
		// if Car button pushed...
			// handle_vehicle_button
    	
		// Execute the correct TLC
		lcd_set_mode(mode + 1);
    	switch (mode) {
			case 0:
				simple_tlc(&proc_state[0]);
//				printf("After proc state: %d\n", proc_state[0]);
				break;
			case 1:
				pedestrian_tlc(&proc_state[1]);
				break;
			case 2:
				configurable_tlc(&proc_state[2]);
				break;
			case 3:
				camera_tlc(&proc_state[3]);
				break;
		}
		// Update Displays
	}
	fclose(lcd);
	return 1;
}
