/*
changed
*/
//#define BTSerial BTSerial

#include <QTRSensors.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#define lmb 3
#define rmb 10
#define lmf 9
#define rmf 6
#define lpwm 3
#define rpwm 5
#define CALIBRATE_BUTTON 8
#define START_BUTTON 4
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   5     // emitter is controlled by digital pin 10

// sensors 0 through 7 are connected to digital pins 11 through 19, except 13 respectively
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5,11,12},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
//unsigned int digitalsensorValues[NUM_SENSORS];
unsigned int position;
bool ctrl=0;
float kp=0.00f;//1
float ki=0.00f;//2
float kd=0.00f;//3
float sp=0.00f;//4
float kr=0.00f;//5
int maxspeed=0;
int mspeed=0;
int increment;
const int smooth_threshold=100;
const int time_threshold=1000;
//int upper_read,lower_read;
int mode;
const int full_rev_delay=2000;
int error, preverror;
float p;
float i;
float d;
float correction_term;
int Speed=0, turn=0, manual_control=0;
//enum{LINE,NOLINE,RLINE,LLINE,BLACK,MANUAL}mode;
void motors_write(int left, int right);
void readval();
void correction();
void read_const();
void write_const();
void bluetooth_receive();
void remote_control();
void manual_flush();
void auto_calibrate();
void manual_calibrate();
void classic_calibrate();
bool todigital(int i);
void send_settings();
void(* reset) (void) = 0;
void detect();
void turn_right();
void turn_left();
void u_turn();
#define L 0
#define R 1
#define T 2
#define LS 3
#define RS 4
#define C 5




SoftwareSerial BTSerial(8,2);


class Timer
{
  	private:
  	unsigned long t;
  	int controls=0;
  	unsigned long t_prev;
  	unsigned long periods;
  	public:
  	void start();
  	void reset()
  	{
  	  t_prev=millis();
  	}
  	void halt()
  	{
  	  controls=0;
  	}
  	unsigned long elapsed();
}t1;
void Timer :: start()
{
	if(!controls)
	{
		t_prev=millis();
		controls++;
	}
}
unsigned long Timer :: elapsed()
{
	if(controls)
 		t=millis();
  	periods=t-t_prev;
  	return periods;
}



void test()
{
	while(1)
  {
    motors_write(0,0);
  }
}



void setup()
{
  //test();
  //qtrrc.calibrate();
  BTSerial.begin(9600);
  Serial.begin(9600);
  pinMode(lmf,OUTPUT);
  pinMode(lmb,OUTPUT);
  pinMode(rmf,OUTPUT);
  pinMode(rmb,OUTPUT);
  pinMode(lpwm,OUTPUT);
  pinMode(rpwm,OUTPUT);
  pinMode(START_BUTTON,INPUT);
  pinMode(CALIBRATE_BUTTON,INPUT);
  pinMode(13,OUTPUT);


	digitalWrite(13,LOW);
	preverror=0;
	i=0;  
	read_const();
	send_settings();
  
}

void loop()//add modes if required
{  
    test();
    readval();
    //detect();
  	bluetooth_receive();
    motors_write(0,0);
  	while(ctrl)
  	{
      readval();
	    correction();
      //detect();
	    bluetooth_receive();//or just BTSerial receive
    }
}
void correction()
{

  
	mspeed=maxspeed;

	error=3500-position;
  //BTSerial.println(error);
	p=error;
  p/=600;
	i+=error;
  i/=5;
	d=error-preverror;
  d/=800;
	correction_term=p*kp+i*ki+d*kd;
	preverror=error;
//Disabled
  /* 
	if(d<smooth_threshold)
	{
		t1.start();
		if(t1.elapsed()>time_threshold)
		{
			mspeed=maxspeed+increment;
			increment++;
		}
		if(mspeed>255)
			mspeed=255;
	}
	else
	{
		t1.halt();
		mspeed=maxspeed;
	}

 */
	int lspeed;
  	int rspeed;
	if(correction_term<0)
	{
    	lspeed=mspeed+correction_term;
    	rspeed=mspeed-(kr*correction_term);
    	if(lspeed>255)
    	  lspeed=255;
    	else if(lspeed<-255)
    	  lspeed=-255;
    	if(rspeed>255)
    	  rspeed=255;
    	else if(rspeed<-255)
    	  rspeed=-255;
		  motors_write(lspeed,rspeed);
	}
	else
	{
		lspeed=mspeed+(kr*correction_term);
    	rspeed=mspeed-correction_term;
    	if(lspeed>255)
    	  lspeed=255;
    	else if(lspeed<-255)
    	  lspeed=-255;
    	if(rspeed>255)
    	  rspeed=255;
    	else if(rspeed<-255)
    	  rspeed=-255;
    	motors_write(lspeed,rspeed);
	}
}
void motors_write(int left=0, int right=0)//switched
{
	if(right>0)
	{
		//digitalWrite(rmb,LOW);
		//digitalWrite(rmf,HIGH);
		analogWrite(rmf, right);
	}
	else
	{
		//digitalWrite(rmf, LOW);
		//digitalWrite(rmb, HIGH);
		analogWrite(rmb, abs(right));
	}
	if(left>0)
	{
		//digitalWrite(lmb,LOW);
		//digitalWrite(lmf,HIGH);
		analogWrite(lmf, left);
	}
	else
	{
		//digitalWrite(lmf, LOW);
		//digitalWrite(lmb, HIGH);
		analogWrite(lmb, abs(left));
	}
}
void readval()
{
	position = qtrrc.readLine(sensorValues,QTR_EMITTERS_ON,1);
}

bool todigital(int i)
{
	int middle=(qtrrc.calibratedMinimumOn[i]+qtrrc.calibratedMaximumOn[i])/2;
	if(sensorValues[i]>middle)
		return 1;
	else
		return 0;
}
void manual_calibrate()
{
	digitalWrite(13,HIGH);
	for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  	{
    	qtrrc.calibrate();       // reads all sensors 10 times at 250 us per read (i.e. ~2.5 ms per call)
  	}
  	write_const();  
  	digitalWrite(13, LOW);
}
void auto_calibrate()
{
	digitalWrite(13,HIGH);
	motors_write(128,-128);
	for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  	{
    	qtrrc.calibrate();       // reads all sensors 10 times at 250 us per read (i.e. ~25 ms per call)
  	}
  	delay(50);
    write_const(); 
  	readval();
    int t=millis();
  	while(!(todigital(3)||todigital(4))&&millis()-t<4000)
  		motors_write(0,0);
  	digitalWrite(13, LOW);
}
void classic_calibrate()
{
	digitalWrite(13,HIGH);
	readval();
	long upper_sum=0,lower_sum=0;
	for(int i=0;i<50;i++)
	{
		upper_sum+=(sensorValues[3]+sensorValues[4])/2;//middle sensors on black
		lower_sum+=(sensorValues[0]+sensorValues[7])/2;//side sensors on black
	}
	int upper_read=upper_sum/50;
	int lower_read=lower_sum/50;
	for(int i=0;i<8;i++)
	{
		qtrrc.calibratedMaximumOn[i]=upper_read;
		qtrrc.calibratedMinimumOn[i]=lower_read;
	}
	digitalWrite(13,LOW);
  	write_const(); 
	//BTSerial.print(upper_read);
	//BTSerial.print(", ");
	//BTSerial.println(lower_read);
}
void read_const()
{

	int add=0;
	//BTSerial.println("read");
	EEPROM.get(add,kp);
	add+=sizeof(float);
	EEPROM.get(add,ki);
	add+=sizeof(float);
	EEPROM.get(add,kd);
	add+=sizeof(float);
	EEPROM.get(add,sp);
	add+=sizeof(float);
	EEPROM.get(add,kr);
	add+=sizeof(float);
	EEPROM.get(add,maxspeed);
	add+=sizeof(int);
	EEPROM.get(add,mode);
	add+=sizeof(int);
 //disabled
 /*
	for(int i=0;i<8;i++)
	{
	  EEPROM.get(add,qtrrc.calibratedMinimumOn[i]);
	  add+=sizeof(int);
	  EEPROM.get(add,qtrrc.calibratedMaximumOn[i]);
	  add+=sizeof(int);
	}
 */
	//BTSerial.println("Read");
}
void write_const()
{
	//BTSerial.println("written");
	int add=0;
	EEPROM.put(add,kp);
	add+=sizeof(float);
	EEPROM.put(add,ki);
	add+=sizeof(float);
	EEPROM.put(add,kd);
	add+=sizeof(float);
	EEPROM.put(add,sp);
	add+=sizeof(float);
	EEPROM.put(add,kr);
	add+=sizeof(float);
	EEPROM.put(add,maxspeed);
	add+=sizeof(int);
	EEPROM.put(add,mode);
	add+=sizeof(int);

 /*
	for(int i=0;i<8;i++)
	{
	  EEPROM.put(add,qtrrc.calibratedMinimumOn[i]);
	  add+=sizeof(int);
	  EEPROM.put(add,qtrrc.calibratedMaximumOn[i]);
	  add+=sizeof(int);
	}
 */
}
void bluetooth_receive()
{
	if(BTSerial.available()>0)
	{
		char test=BTSerial.read();
		switch(test)
		{
			case 'r':
					manual_control=1;
					remote_control();
					break;
			case 'a':
					manual_control=0;
					break;
			case 'm':
            //BTSerial.println("motor var received!");
						Speed=BTSerial.readStringUntil(',').toInt()-255;
						turn=(BTSerial.readStringUntil(',').toInt()-255)/8;
					  break;
			case 's':
						kp=BTSerial.readStringUntil(',').toFloat();
						ki=BTSerial.readStringUntil(',').toFloat();
						kd=BTSerial.readStringUntil(',').toFloat();
						kr=BTSerial.readStringUntil(',').toFloat();
						sp=BTSerial.readStringUntil(',').toFloat()-4;
						maxspeed=BTSerial.readStringUntil(',').toInt();
					write_const();
					break;
			case 'c':
						manual_calibrate();
						break;
			case 'x':
						reset();
						break;
			case 'i':
						i=0;
						break;
      case 'b':
            mode=BTSerial.readStringUntil(',').toInt();
            break;
      case 'd':
            ctrl=!ctrl;
            break;
			default:
					manual_flush();
		}
	}
}
void remote_control()
{
  	int lspeed,rspeed;
	while(manual_control)
	{
    rspeed=Speed-turn;
    lspeed=Speed+turn;
    if(rspeed>255)
      rspeed=255;
    else if(rspeed<-255)
      rspeed=-255;
    if(lspeed>255)
      lspeed=255;
    else if(lspeed<-255)
      lspeed=-255;
  
		motors_write(lspeed,rspeed);
		bluetooth_receive();
	}

}
void manual_flush()
{
	while(BTSerial.available()>0)
	{
		char a=BTSerial.read();
	}
}

void send_settings()
{
	BTSerial.print("Kp= ");
	BTSerial.print(kp);
	BTSerial.print("Ki= ");
	BTSerial.print(ki);
	BTSerial.print("Kd= ");
	BTSerial.print(kd);
	BTSerial.print("Kr= ");
	BTSerial.print(kr);
	BTSerial.print("SP= ");
	BTSerial.print(sp);
	BTSerial.print("Maxspeed= ");
	BTSerial.println(maxspeed);
 	BTSerial.print("Mode= ");
 	BTSerial.println(mode);

}
void detect()
{
  uint8_t state=0;
  for(int i=0;i<8;i++)
  {
    state<<=state;
    state+=todigital(i);
  }
  switch(state)//add error tolerances
  {
    case B00000000://U turn
      //ctrl=0;
      u_turn();
      break;
    case B11111111://T-point check for straight
      turn_left();
      break;
    case B11110000://Left turn
    case B11100000:
      turn_left();
      break;
    case B00001111:
    case B00000111:
      turn_right();
      break;
  }
}
void turn_left()
{
  readval();
  while(todigital(3)!=1)
  {
    motors_write(maxspeed/2,maxspeed);
    readval();
  }
}
void turn_right()
{
  readval();
  while(todigital(4)!=1)
  {
    motors_write(maxspeed,maxspeed/2);
    readval();
  }
}
void u_turn()
{
  readval();
  while(todigital(3)!=1)
  {
    motors_write(-maxspeed,maxspeed);
    readval();
  }
  
}
