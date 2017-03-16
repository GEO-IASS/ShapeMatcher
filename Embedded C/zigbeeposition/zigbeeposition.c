
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function

unsigned char data[4];
unsigned int count = 0;

unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int left_motor_pulse_count_locked = 0x00;
unsigned int Degrees; //to accept angle in degrees for turning

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;

float BATT_Voltage, BATT_V;

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//configure spi pins to talk to secondary microcontroller
void spi_pin_config (void)
{
 DDRB = DDRB | 0x07;
 PORTB = PORTB | 0x07;
}

//MOSFET switch port configuration
void MOSFET_switch_config (void)
{
 DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
 PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

 DDRG = DDRG | 0x04; //make PORTG 2 pin as output
 PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

//Function to initialize ports
void port_init()
{
 motion_pin_config(); //robot motion pins config
 buzzer_pin_config(); //buzzer pins config
 adc_pin_config();	//adc pins config
 spi_pin_config(); //initialize spi interface
 MOSFET_switch_config(); //configure MOSFETs
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
}


//SPI initialize
// clock rate: 921600hz
void spi_init(void)
{
 SPCR = 0x53; //setup SPI
 SPSR = 0x00; //setup SPI
 SPDR = 0x00;
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	unsigned char t;
	t = Ch;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	if(t<9 && t>0)
	{
		UDR0 = a;
	}
	return a;
}

//-------------------------------------------------------------------------------
//-- Function To Transmit/Receive through SPI interface ------
//-------------------------------------------------------------------------------
unsigned char spi_master_tx_and_rx (unsigned char data)
{
 unsigned char rx_data = 0;

 PORTB = PORTB & 0xFE;        // make SS pin low
 SPDR = data;
 while(!(SPSR & (1<<SPIF)));  //wait for data transmission to complete

 _delay_ms(1);                //time for ADC conversion in the slave microcontroller
 
 SPDR = 0x50;                 // send dummy byte to read back data from the slave microcontroller
 while(!(SPSR & (1<<SPIF)));  //wait for data reception to complete
 rx_data = SPDR;
 PORTB = PORTB | 0x01;        // make SS high
 UDR0 = rx_data;
 return rx_data;
}



// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
	UDR0 = 0x00;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void) //stops robot motion
{
  motion_set(0x00);
}

void turn_on_sharp234_wl (void) //turn on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
  PORTG = PORTG & 0xFB;
}

void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
 PORTG = PORTG | 0x04;
}

void turn_on_sharp15 (void) //turn on Sharp IR range sensors 1,5
{
  PORTH = PORTH & 0xFB;
}

void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
 PORTH = PORTH | 0x04;
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
 PORTH = PORTH & 0xF7;
 UDR0 = 0x00;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
 PORTH = PORTH | 0x08;
 UDR0 = 0x00;
}

void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
									  // Sharp 1, 5 and IR proximity sensor
{
 PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
 PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
 UDR0 = 0x00;
}

void turn_off_all_proxy_sensors (void) // turn off Sharp 2, 3, 4, red LED of the white line sensors
									  // Sharp 1, 5 and IR proximity sensor
{
 PORTH = PORTH | 0x0C; //set PORTH 3 and PORTH 1 pins to 1
 PORTG = PORTG | 0x04; //set PORTG 2 pin to 1
 UDR0 = 0x00;
}

//Function to turn on the buzzer
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
 UDR0 = 0x00;
}

//Function to turn off the buzzer
void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
 UDR0 = 0x00;
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
 UDR0 = 0x00;
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
 UDR0 = 0x00;
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}


void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
 PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
 UDR0 = 0x00;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
 UDR0 = 0x00;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
 UDR0 = 0x00;
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
 UDR0 = 0x00;
}

//Function to initialize all the devices
void init_devices()
{
 cli(); //Clears the global interrupt
 port_init();  //Initializes all the ports
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 uart0_init(); //Initailize UART0 for serial communiaction
 timer1_init();
 timer5_init();
 adc_init();
 spi_init();
 sei();   // Enables the global interrupt 
}

ISR(USART0_RX_vect) 		// ISR for receive complete interrupt
{
	data[count] = UDR0;	
	count = count + 1;

	while(count == 4)
	{	
		if(data[0] == 0x10)
		{
			sei();
			stop();
			count = 0;
			break;
		}
		if(data[0] == 0x11)
		{	
			sei();
				if(data[2] == 0x01 )
				{
					unsigned int x = (int)data[1];
				  	forward_mm(x*8);
				 }
				 else if(data[2] == 0x00)
				 {
				 	forward();
				 }
			count = 0;
			break;			
		}
		else if(data[0] == 0x12)
		{
			sei();
				if(data[2] == 0x01 )
				{
					unsigned int x = (int)data[1];
					back_mm(x*8);
				}
				else if(data[2] == 0x00)
				{
					back();
				}
			count = 0;
			break;	
		}
		else if(data[0] == 0x13)
		{
			sei();
			unsigned int x = (int)data[1];
			right_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0x14)
		{
			sei();
			unsigned int x = (int)data[1];
			left_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0x15)
		{
			unsigned int temp;
			sei();
			temp = ADC_Conversion(data[1]);
			count = 0;
			break;	
		}
		else if(data[0] == 0x16)
		{	
			sei();
			float temp;
			unsigned int converted;
			sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp);	
			temp = ((value*255.00)/800.00);
			converted = (int)temp;
			unsigned char templates = (char)converted;
			UDR0 = templates;
			count = 0;
			break;
		}
		else if(data[0] == 0x17)
		{
			sei();
			if(data[1] == 1)
			{
				servo_1(data[2]);
			}
			else if(data[1] == 2)
			{
				servo_2(data[2]);
			}
			count = 0;
			break;
		}
		else if(data[0] == 0x18)
		{
			sei();
			velocity(data[1],data[2]);
			count = 0;
			break;
		}
		else if(data[0] == 0x19)
		{
			sei();
			if(data[1] == 1)
			{
				buzzer_on();
			}
			if(data[1] == 0)
			{
				buzzer_off();
			}
			count = 0;
			break;
		}
		else if(data[0] == 0x1A)
		{
			unsigned int temp;
			sei();
			temp = spi_master_tx_and_rx(data[1]);
			count = 0;
			break;
		}
		else if(data[0] == 0x1B)
		{
			sei();
			unsigned int x = (int)data[1];
			soft_right_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0x1C)
		{
			sei();
			unsigned int x = (int)data[1];
			soft_left_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0x1D)
		{
			sei();
			unsigned int x = (int)data[1];
			soft_right_2_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0x1E)
		{
			sei();
			unsigned int x = (int)data[1];
			soft_left_2_degrees(x*2);
			count = 0;
			break;	
		}
		else if(data[0] == 0X1F)
		{
			sei();
			turn_off_all_proxy_sensors();
			turn_off_ir_proxi_sensors();
			count = 0;
			break;
		}
		else if(data[0] == 0X20)
		{
			sei();
			turn_on_all_proxy_sensors();
			turn_on_ir_proxi_sensors();
			count = 0;
			break;
		}
		else if(data[0] == 0x21)
		{ 
			sei();
			float temp;
			unsigned int converted;
			unsigned int Batt;
			BATT_V = ADC_Conversion(0);
			BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;	//Prints Battery Voltage Status
			Batt = (int)BATT_Voltage;
			temp = ((Batt*255.00)/1040.00);
			converted = (int)temp;
			unsigned char templates = (char)converted;
			UDR0 = templates;
			count = 0;
			break;
		}
		
	}
}


//Main Function

int main(void)
{
	init_devices();
	servo_1(90);
	servo_2(70);
	while(1);
}
