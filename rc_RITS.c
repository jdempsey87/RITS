/*******************************************************************************
* rc_RITS.c
*
* This is meant to be a skeleton program for robotics cape projects. 
* Change this description and file name 
* 
* 
*******************************************************************************/

// usefulincludes is a collection of common system includes for the lazy
// This is not necessary for roboticscape projects but here for convenience
#include <rc_usefulincludes.h> 
// main roboticscape API header
#include <roboticscape.h>
#include <inttypes.h>

// choice of 1,2,4,8,16 oversampling. Here we use 16 and sample at 25hz which
// is close to the update rate specified in robotics_cape.h for that oversample.
#define OVERSAMPLE  BMP_OVERSAMPLE_16
// choice of OFF, 2, 4, 8, 16 filter constants. Here we turn off the filter and 
// opt to use our own 2nd order filter instead.
#define INTERNAL_FILTER	BMP_FILTER_OFF

// our own low pass filter
#define ORDER			2
#define CUTOFF_FREQ		2.0f	// 2rad/s, about 0.3hz
#define BMP_CHECK_HZ	25
#define	DT				1.0f/BMP_CHECK_HZ

// possible modes, user selected with command line arguments
typedef enum m_mode_t{
	RAD,
	DEG,
	RAW
} m_mode_t;

//struct to hold new data
rc_imu_data_t data; 

//variables 

int baroMode= 0;

//baro
double temp, pressure, altitude, filtered_alt;
rc_filter_t lowpass;

//dmp
int c, sample_rate, priority;
int show_something = 0; // set to 1 when any show data option is given.
//imu configuration
rc_imu_config_t conf;
//imu mode
	m_mode_t mode;
	
	
// function declarations
void on_pause_pressed();
void on_pause_released();
void print_usage();

/*******************************************************************************
* void print_usage()
*
* Printed if some invalid argument was given, or -h option given.
*******************************************************************************/
void print_usage(){
	printf("\n Options\n");
	printf("-r		print raw values instead of radians\n");
	printf("-d		print gyro in degrees/s instead of radians\n");
	printf("-h		Print this help message\n\n");
	
	return;
}

/*******************************************************************************
* void print_header()
*
* Based on which data is marked to be printed, print the correct labels.
* this is printed only once and the actual data is updated on the next line.
*******************************************************************************/
void print_header(){
	// print my header	
	printf("\n**********************************\n");
	printf("\n");
	printf("\n	       RITS         \n");
	printf("\n");
	printf("\n**********************************\n");
	printf("\n");
	printf("\n");
	
		printf("\ntry 'rc_RITS -h' to see other options\n\n");
	
	// print a header
	printf("  temp  |");
	printf(" pressure  |");
	printf(" altitude |");
	printf(" filtered |");
	
	switch(mode){
	case RAD:
		printf("   Accel XYZ(m/s^2)  |");
		printf("   Gyro XYZ (rad/s)  |");
		break;
	case DEG:
		printf("   Accel XYZ(m/s^2)  |");
		printf("   Gyro XYZ (deg/s)  |");
		break;
	case RAW:
		printf("  Accel XYZ(raw adc) |");
		printf("  Gyro XYZ (raw adc) |");
		break;
	default:
		printf("ERROR: invalid mode\n");
		return -1;
	}
	printf("  Mag Field XYZ(uT)  |");
	printf(" Temp (C)");
	printf("\n");
	
	// printf("FilteredComp |");
	// printf("   Fused Quaternion  |");
	// printf(" FusedTaitBryan(deg) |");
	// printf("   Accel XYZ (g)   |");
	// printf("  Gyro XYZ (deg/s) |");
//	printf(" Temp(C) |");
	
	printf("\n");
}

/*******************************************************************************
* void print_data()
*
* This is the IMU interrupt function.  
*******************************************************************************/
void print_data(){
	printf("\r");
	printf(" ");
				printf("\r");
		//printf(" ");
				
			printf("%6.2fC |", temp);
			printf("%7.2fkpa |", pressure/1000.0);
			printf("%8.2fm |", altitude);
			printf("%8.2fm |", filtered_alt);
			
			
			
				// print accel
		if(rc_read_accel_data(&data)<0){
			printf("read accel data failed\n");
		}
		if(mode==RAW){
			printf("%6d %6d %6d |",			data.raw_accel[0],\
											data.raw_accel[1],\
											data.raw_accel[2]);
		}
		else{
			printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
											data.accel[1],\
											data.accel[2]);
		}
		
		// print gyro data
		if(rc_read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}
		switch(mode){
		case RAD:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0]*DEG_TO_RAD,\
											data.gyro[1]*DEG_TO_RAD,\
											data.gyro[2]*DEG_TO_RAD);
			break;
		case DEG:
			printf("%6.1f %6.1f %6.1f |",	data.gyro[0],\
											data.gyro[1],\
											data.gyro[2]);
			break;
		case RAW:
			printf("%6d %6d %6d |",			data.raw_gyro[0],\
											data.raw_gyro[1],\
											data.raw_gyro[2]);
			break;
		default:
			printf("ERROR: invalid mode\n");
			return -1;
		}

		// read magnetometer
		if(rc_read_mag_data(&data)<0){
			printf("read mag data failed\n");
		}
		else printf("%6.1f %6.1f %6.1f |",	data.mag[0],\
											data.mag[1],\
											data.mag[2]);

		// read temperature
		if(rc_read_imu_temp(&data)<0){
			printf("read temp data failed\n");
		}
		else printf(" %4.1f ", data.temp);
						
			
			fflush(stdout);		


		// printf("   %6.1f   |", data.compass_heading*RAD_TO_DEG);
	

		// printf(" %4.1f %4.1f %4.1f %4.1f |", 	data.fused_quat[QUAT_W], \
		// 										data.fused_quat[QUAT_X], \
		// 										data.fused_quat[QUAT_Y], \
		// 										data.fused_quat[QUAT_Z]);
		// printf("%6.1f %6.1f %6.1f |",	data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
		// 								data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
		// 								data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);


		// printf(" %5.2f %5.2f %5.2f |",	data.accel[0],\
		// 								data.accel[1],\
		// 								data.accel[2]);

		// printf(" %5.1f %5.1f %5.1f |",	data.gyro[0],\
		// 								data.gyro[1],\
		// 								data.gyro[2]);
	
	
		//rc_mpu_read_temp(&data);
		//printf(" %6.2f |", data.temp);
	fflush(stdout);
	return;
}

/*******************************************************************************
* int main() 
*
* This template main function contains these critical components
* - call to rc_initialize() at the beginning
* - main while loop that checks for EXITING condition
* - rc_cleanup() at the end
*******************************************************************************/
int main(int argc, char *argv[]){
	lowpass = rc_empty_filter();
	
		mode = RAD; // default to radian mode.

	
	
		// parse arguments
	opterr = 0;
	while ((c=getopt(argc, argv, "s:magrqtcp:hwo"))!=-1 && argc>1){
		switch (c){
		
			break;
		case 'h': // show help option
			print_usage();
			return -1;
			break;
		case 'r':
			if(mode!=RAD) print_usage();
			mode = RAW;
			printf("\nRaw values are from 16-bit ADC\n");
			break;
		case 'd':
			if(mode!=RAD) print_usage();
			mode = DEG;
			break;
		default:
			printf("opt: %c\n",c);
			printf("invalid argument\n");
			print_usage();
			return -1;
			break;
		}
	}
	
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}
	
		// use defaults for now, except also enable magnetometer.
	conf = rc_default_imu_config();
	conf.enable_magnetometer=1;
	
	//init imu
		if(rc_initialize_imu(&data, conf)){
		fprintf(stderr,"rc_initialize_imu_failed\n");
		return -1;
	}
	
	//initialize baro
	if(rc_initialize_barometer(OVERSAMPLE, INTERNAL_FILTER)<0){
		fprintf(stderr,"ERROR: rc_initialize_barometer failed\n");
		return -1;
	}
	// create the lowpass filter and prefill with current altitude
	if(rc_butterworth_lowpass(&lowpass,ORDER, DT, CUTOFF_FREQ)){
		fprintf(stderr,"ERROR: failed to make butterworth filter\n");
		return -1;
	}
	altitude = rc_bmp_get_altitude_m();
	rc_prefill_filter_inputs(&lowpass, altitude);
	rc_prefill_filter_outputs(&lowpass, altitude);
	
	// // now set up the imu for dmp interrupt operation
	// if(rc_initialize_imu_dmp(&data, conf)){
	// 	printf("rc_initialize_imu_failed\n");
	// 	return -1;
	// }

// do my initialization here

	
print_header();

	//rc_set_imu_interrupt_func(&print_data);
	//now just wait, print_data() will be called by the interrupt
	
	
	rc_set_pause_pressed_func(&on_pause_pressed);
	rc_set_pause_released_func(&on_pause_released);

	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		rc_usleep(1000000/BMP_CHECK_HZ);
		// handle other states
		if(rc_get_state()==RUNNING){
			// do things
			rc_set_led(GREEN, ON);
			rc_set_led(RED, OFF);
			
			// perform the i2c reads to the sensor, this takes a bit of time
			if(rc_read_barometer()<0){
				fprintf(stderr,"\rERROR: Can't read Barometer");
				fflush(stdout);
				continue;
			}
			// if we got here, new data was read and is ready to be accessed.
			// these are very fast function calls and don't actually use i2c
			temp = rc_bmp_get_temperature();
			pressure = rc_bmp_get_pressure_pa();
			altitude = rc_bmp_get_altitude_m();
			filtered_alt = rc_march_filter(&lowpass,altitude);
	

	
	
			print_data();
	

			
			
		}else if(rc_get_state()==PAUSED){
			// do other things
			rc_set_led(GREEN, OFF);
			rc_set_led(RED, ON);
		}
		// always sleep at some point
		
		

	}
	
	// exit cleanly
	rc_power_off_barometer();
		// shut things down
	rc_power_off_imu();
	rc_cleanup(); 
	return 0;
}


/*******************************************************************************
* void on_pause_released() 
*	
* Make the Pause button toggle between paused and running states.
*******************************************************************************/
void on_pause_released(){
	// toggle betewen paused and running modes
	if(rc_get_state()==RUNNING)		rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/*******************************************************************************
* void on_pause_pressed() 
*
* If the user holds the pause button for 2 seconds, set state to exiting which 
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
void on_pause_pressed(){
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds
	
	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_get_pause_button() == RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
