/*
    Name		:	ADS1115_driver.ino
    Created		:	19-Jan-19 17:00
    Author		:	Oscar goos
	Code		:	C++, Flash: 8872B, RAM: 494B
	Contrcoller	:	ATmega328
	Last tested	:	14-Jan-2019
	Notes		:   
				:	
				:	
				:	
				:	
				:	
*/

#include		<Wire.h>

#define			A_B				0
#define			A_D				1
#define			B_D				2
#define			C_D				3
#define			A				4
#define			B				5
#define			C				6
#define			D				7

#define			I2C_ADS1115_ADR	0x48	// 0b1001000, 0x48, 0x49,0x4A, 0x4B
#define			I2C_ADS1115_ID	0x00	// ID= 0,1,2,3, A0= gnd, Vdd, SDA, SCL

#define			ADC1115			0x00
#define			CONFIG			0x01
#define			THRES_LO		0x02
#define			THRES_HI		0x03
#define			CONVERT			0x04
#define			READ			0x06
#define			WRITE			0x07


#define			INITADC			0x01


#define			STOP			true
#define			CLOCKFREQ		400000

typedef struct ADS1115reg_t {
	uint8_t		COMP_Q	 : 2;		// Assert RDY after <COMP_Q=0,1,2> conversions, COMP_Q=3 diable comparator
	uint8_t		COMP_L	 : 1;		// Latch/Nop latch RDY 
	uint8_t		COMP_P	 : 1;		// Polarity of RDY, COMP_P=0 RDY active low, COMP_P=1 RDY active high, 
	uint8_t		COMP_M	 : 1;		// Comperator Mode, COMP_M=0 normal compearator, COMP_M=1 Window comparator
	uint8_t		DRate	 : 3;		// Conversion rate: DRate=0..7 = 8,16,32,64,128,250,475,860 SPS	

	uint8_t		MODE	 : 1;		// Continues: Mode=0, Single shot or Power down state: mode=1, 
	uint8_t		PGA		 : 3;		// Prog Gain Amp=0..7, FSR 6.144V, 4.096V, 2.048V. 1.024V, 0.512V, 0.256V, 0.256V, 0.256V	
	uint8_t		MUX		 : 3;		// MUX=0,1,2,3, Differental mode between: Ain0-Ain1, Ain0-Ain3, Ain1-Ain3, Ain2-Ain3,  MUX=4,5,6,7 Single ended Mode
	uint8_t		OS		 : 1;		// Depending of MODE, WR: 0= Do nothing 1=Start a Single shot,  RD: 0=performing a conversion 1=Not performing a conversion

	uint8_t		alert	 : 1;		// bit 16 of Threshold High and low register. To enable ThresH_Lo:16=0, ThresH_Hi:16=1

	uint16_t	ConfReg		;		// 
	uint16_t	ThresH_Lo	;		// Two complement 15bit value always smaller than ThresH_Hi
	uint16_t	ThresH_Hi	;		// Two complement 15bit value always bigger  than ThresH_Lo
	int16_t		ch[8]		;		// last two complement Conversion values for channel A..D
}ADS1115reg_t;
ADS1115reg_t	ADS = { 0x03, 0x00, 0x00, 0x00, 0x07,     0x00, 0x00, B, 0x01,     0x01,  0x0000, 0x7FFF, 0x8000, { 0,0,0,0,0,0,0,0 } };

typedef struct DAC_t {
	float		UDACref, UADCref;				// UDACref = 2.048, Vdd, UADCref= 1.1, Vdd, DACch3; 
	uint8_t		GDAC, GAMP;						// GDAC = 1,2 ,  GAMP=2;
	float		GRES, GADCcal, GDACcal;			// Gres = resistor devider in front of the ADC= 0.31973, GDACcal & GADCcal= 1 is calibrartie factor dor fine tuning DACout and ADCin 
	int16_t		DCM, ACM;						// DCM=DACcode_maximum = 4095, ACM=ADCcode_maximum=1023. 

}DAC_t;
DAC_t			D2A = { 5.01, 6.144, 1, 1, 1, 1, 1, 0x0FFF, 0x7FFF };

uint16_t		aval;

void setup()
{
	Serial.begin(115200);
	Wire.begin();

	ADS1115_Driver(INITADC, WRITE);
	Serial.print(ADS.ConfReg,HEX);Serial.print(",     ");Serial.println(ADS.ConfReg,BIN);
	Serial.print("ADS.OS \t\t= ");		Serial.println(ADS.OS,HEX);
	Serial.print("ADS.MUX \t= ");		Serial.println(ADS.MUX,HEX);
	Serial.print("ADS.PGA \t= ");		Serial.println(ADS.PGA,HEX);
	Serial.print("ADS.MODE \t= ");		Serial.println(ADS.MODE,HEX);
	Serial.print("ADS.DRate \t= ");		Serial.println(ADS.DRate,HEX);
	Serial.print("ADS.COMP_M \t= ");		Serial.println(ADS.COMP_M,HEX);
	Serial.print("ADS.COMP_P \t= ");		Serial.println(ADS.COMP_P,HEX);
	Serial.print("ADS.COMP_L \t= ");		Serial.println(ADS.COMP_L,HEX);
	Serial.print("ADS.COMP_Q \t= ");		Serial.println(ADS.COMP_Q,HEX);

	ADS1115_Driver(CONFIG, READ);
	Serial.print(ADS.ConfReg,HEX);Serial.print(",     ");Serial.println(ADS.ConfReg,BIN);
	Serial.print("ADS.OS \t\t= ");		Serial.println(ADS.OS,HEX);
	Serial.print("ADS.MUX \t= ");		Serial.println(ADS.MUX,HEX);
	Serial.print("ADS.PGA \t= ");		Serial.println(ADS.PGA,HEX);
	Serial.print("ADS.MODE \t= ");		Serial.println(ADS.MODE,HEX);
	Serial.print("ADS.DRate \t= ");		Serial.println(ADS.DRate,HEX);
	Serial.print("ADS.COMP_M \t= ");		Serial.println(ADS.COMP_M,HEX);
	Serial.print("ADS.COMP_P \t= ");		Serial.println(ADS.COMP_P,HEX);
	Serial.print("ADS.COMP_L \t= ");		Serial.println(ADS.COMP_L,HEX);
	Serial.print("ADS.COMP_Q \t= ");		Serial.println(ADS.COMP_Q,HEX);

	delay(3000);
}


void loop()
{
//	ADS1115_Driver(CONVERT, B);			// not needed in continues mode.
	delay(100);
	ADS1115_Driver(ADC1115, B);
	aval = round(0x0FFF);
//	Serial.println((String)"DAC out:=" + (float)(D2A.UDACref*D2A.GDAC*D2A.GAMP*D2A.GDACcal*aval) / (D2A.DCM) + "\t ADC in:= " + String((float)(D2A.UADCref*ADS.ch[B])/(float)(D2A.ACM * D2A.GRES),4) + ",  " + String(ADS.ch[B],HEX));
	Serial.println(String("ADC in:= ") + String(ADS.ch[B]) + ",\t" + String(ADS.ch[B],HEX)); 
	delay(900);
}



uint8_t ADS1115_Driver(uint8_t cmd, uint8_t par) {		
	
	uint8_t	page, chan, spar;


	Wire.beginTransmission(I2C_ADS1115_ADR);

	switch (cmd) {
	
		case ADC1115: {										// read the ADC value from conversion register 
			Wire.write(cmd);
			Wire.endTransmission();
			Wire.requestFrom(I2C_ADS1115_ADR, 2);
			ADS.ch[par] = (int16_t)(Wire.read() << 8 | Wire.read());
			Wire.endTransmission();
		} break;

		case CONVERT: {										// Prepare new chanel and start a convertion
			ADS.MUX = par;
			ADS.OS = 1;
			cmd = CONFIG;
			par = WRITE;
		}	// fall through to exeute CONFIG

		case CONFIG: {										// Write or Read the Configuration data to and from Config register
			Wire.write(cmd);
			if (par == WRITE) {
				ADS.ConfReg = (uint16_t)(ADS.OS << 15 | ADS.MUX << 12 | ADS.PGA << 9 | ADS.MODE << 8 | ADS.DRate << 5 | ADS.COMP_M << 4 | ADS.COMP_P << 3 | ADS.COMP_L << 2 | ADS.COMP_Q);
				Wire.write((uint8_t)(ADS.ConfReg >> 8));
				Wire.write((uint8_t)(ADS.ConfReg & 0xFF));
				Wire.endTransmission();
				// test ADS1115_Driver(THRES_HI, WRITE);
				// test ADS1115_Driver(THRES_LO, WRITE);
			}
			if (par == READ) {
				Wire.write(cmd);
				Wire.endTransmission();
				Wire.requestFrom(I2C_ADS1115_ADR, 2);
				ADS.ConfReg = (uint16_t)(Wire.read() << 8 | Wire.read());
				ADS.OS		= (ADS.ConfReg >> 15) & 0x0001;
				ADS.MUX		= (ADS.ConfReg >> 12) & 0x0007;
				ADS.PGA		= (ADS.ConfReg >> 9)  & 0x0007;
				ADS.MODE	= (ADS.ConfReg >> 8)  & 0x0001;
				ADS.DRate	= (ADS.ConfReg >> 5)  & 0x0007;
				ADS.COMP_M	= (ADS.ConfReg >> 4)  & 0x0001;
				ADS.COMP_P	= (ADS.ConfReg >> 3)  & 0x0001;
				ADS.COMP_L	= (ADS.ConfReg >> 2)  & 0x0001;
				ADS.COMP_Q	= (ADS.ConfReg >> 0)  & 0x0003;
				Wire.endTransmission();
			}
		} break;

		case THRES_HI: {									// Write the alert High level value to threshold High register
			Wire.write(cmd);
			if (par == WRITE) {
				// add alert to ThresH_Hi
				Wire.write((uint8_t)(ADS.ThresH_Hi >> 8));
				Wire.write((uint8_t)(ADS.ThresH_Hi & 0xFF));
				Wire.endTransmission();
				Wire.requestFrom(I2C_ADS1115_ADR, 2, STOP);
				Serial.println(Wire.read(),BIN);
				Serial.println(Wire.read(),BIN);

			}
		} break;

		case THRES_LO: {									// Write the alert Low level value to threshold Low register
			Wire.write(cmd);
			if (par == WRITE) {
				// add !alert to ThresH_Lo
				Wire.write((uint8_t)(ADS.ThresH_Lo >> 8));
				Wire.write((uint8_t)(ADS.ThresH_Lo & 0xFF));
				Wire.endTransmission();
			}
		} break;

	}
}
