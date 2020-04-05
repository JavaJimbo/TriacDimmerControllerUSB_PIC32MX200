#include <I2C_EEPROM_PIC32.h>
#include <plib.h>

#define USE_I2C1
// #define USE_I2C2

#ifdef USE_I2C1
#define StartI2C() StartI2C1() 
#define StopI2C() StopI2C1()
#define IdleI2C() IdleI2C1()
#define MasterWriteI2C MasterWriteI2C1
#define MasterReadI2C() MasterReadI2C1()
#define MasterputsI2C MasterputsI2C1
#define MastergetsI2C MastergetsI2C1
#define I2CSTATbits I2C1STATbits
#define RestartI2C() RestartI2C1()
#endif

#ifdef USE_I2C2
#define StartI2C() StartI2C2() 
#endif


/*******************************************************************
*	Name:	sendString.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	short address - 16-bit memory address to write the data to
*	unsigned char *data - pointer to data string to be stored
*	int length - length of data string to be stored
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  A string of data (with length bytes) 
*	is sent to the I2C device with SlaveAddress.  Data is written to
*	the specified memory address on the EEPROM using the function
*	MasterputsI2C1, which writes a string of data.  It then waits
*	for an acknowledgement from the EEPROM saying that it has finished
*	writing the data.
*	
*******************************************************************/
void sendString(unsigned char SlaveAddress, short address, unsigned char *data, int length)
{ 
	// Initialize data sending
	char i2cData[3];
	int  DataSz = 3;

	// Send Data to eeprom to program one location

	i2cData[0] = (SlaveAddress << 1) | 0;	//EEPROM Device Address and WR Command
	i2cData[1] = (address>>8);	//eeprom location to program (high address byte)
	i2cData[2] = (address);	//eeprom location to program (low address byte)

	StartI2C();	//Send the Start Bit
	IdleI2C();		//Wait to complete

	int Index = 0;
	while( DataSz )
	{
		MasterWriteI2C( i2cData[Index++] );
		IdleI2C();		//Wait to complete
		DataSz--;

		//ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
		if( I2CSTATbits.ACKSTAT )
			break;
	}

	MasterputsI2C(data);
	StopI2C();	//Send the Stop condition
	IdleI2C();	//Wait to complete

	// wait for eeprom to complete write process
	getWriteAck(SlaveAddress);
} 

/*******************************************************************
*	Name:	sendChar.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	short address - 16-bit memory address to write the data to
*	unsigned char *data - pointer to data string to be stored
*	int length - length of data string to be stored
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  A string of data (with length bytes) 
*	is sent to the I2C device with SlaveAddress.  Data is written to
*	the specified memory address on the EEPROM using the function
*	MasterWriteI2C1, which writes a string of data one character at a time.
*	It then waits for an acknowledgement from the EEPROM saying that it 
*	has finished writing the data.
*	
*******************************************************************/
void sendChar(unsigned char SlaveAddress, short address, unsigned char *data, int length)
{ 
	// Initialize data sending
	char i2cData[3];
	int  DataSz = 3;

	// Send Data to eeprom to program one location

	i2cData[0] = (SlaveAddress << 1) | 0;	//EEPROM Device Address and WR Command
	i2cData[1] = (address>>8);	//eeprom location to program (high address byte)
	i2cData[2] = (address);	//eeprom location to program (low address byte)

	StartI2C();	//Send the Start Bit
	IdleI2C();		//Wait to complete
    
	int Index = 0;
	while (DataSz){
        MasterWriteI2C (i2cData[Index++]);
        IdleI2C();	//Wait to complete
		DataSz--;
		// ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
		if (I2CSTATbits.ACKSTAT)
			break;
	}

	while (length){
		MasterWriteI2C (*data);
		IdleI2C();		// Wait to complete
		data++; // Go to next memory address
		length--;
        
		// ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
		if (I2CSTATbits.ACKSTAT)
			break;
	}
	StopI2C();	//Send the Stop condition
	IdleI2C();	//Wait to complete
	// wait for eeprom to complete write process
	getWriteAck(SlaveAddress);
}

/*******************************************************************
*	Name:	getWriteAck.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  It polls the EEPROM until it recieves an
*	acknowlegement that it has completed the write process.
*	
*******************************************************************/
void getWriteAck(unsigned char SlaveAddress)
{ 
	// wait for eeprom to complete write process. poll the ack status
	while(1){
		//i2c_wait(10);

		StartI2C();	//Send the Start Bit
		IdleI2C();		//Wait to complete

		MasterWriteI2C( (SlaveAddress << 1) | 0 );
		IdleI2C();		//Wait to complete

		if (I2CSTATbits.ACKSTAT == 0){	//eeprom has acknowledged
			StopI2C();	//Send the Stop condition
			IdleI2C();	//Wait to complete
			break;
		}

		StopI2C();	//Send the Stop condition
		IdleI2C();	//Wait to complete 
	}
}

/*******************************************************************
*	Name:	sendReceiveRequest.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	short address - 16-bit memory address to write the data to
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  The read command is sent to the EEPROM
*	to initialize a reading sequence.
*	
*******************************************************************/
void sendReceiveRequest(unsigned char SlaveAddress, short address)
{ 
	char i2cData[3];
	int  DataSz = 3;

	// Now Readback the data from the serial eeprom

	i2cData[0] = (SlaveAddress << 1) | 0;	//EEPROM Device Address and WR Command (to write the address)
	i2cData[1] = address>>8;	//eeprom location to read (high address byte)
	i2cData[2] = address;	//eeprom location to read (low address byte)

	StartI2C();	//Send the Start Bit
	IdleI2C();		//Wait to complete

	//send the address to read from the serial eeprom
	int Index = 0;
	while (DataSz){
		MasterWriteI2C( i2cData[Index++] );
		IdleI2C();		//Wait to complete

		DataSz--;

		// ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
		if (I2CSTATbits.ACKSTAT)
			break;
	}

	//now send a start sequence again
	RestartI2C();	//Send the Restart condition

	//wait for this bit to go back to zero
	IdleI2C();	//Wait to complete

	MasterWriteI2C( (SlaveAddress << 1) | 1 ); //transmit read command
	IdleI2C();		//Wait to complete
}

/*******************************************************************
*	Name:	receiveString.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	short address - 16-bit memory address to read the data from
*	unsigned char *data - pointer to data string to be read
*	int length - length of data string to be read
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  A string of data (with length bytes) 
*	is read from the I2C device with SlaveAddress.  Data is read from
*	the specified memory address on the EEPROM using the function
*	MastergetsI2C1, which reads a string of data all at once.
*	
*******************************************************************/
void receiveString(unsigned char SlaveAddress, short address, unsigned char *data, int length)
{ 
	sendReceiveRequest(SlaveAddress, address);	

	// get data as a string
	MastergetsI2C(length-1, data, 100*length);
	
	// Set the last character to null as it will otherwise be random data from the EEPROM
	data+=length-1;
	*data = 0;
	
	StopI2C();
	IdleI2C();
} 

/*******************************************************************
*	Name:	receiveChar.c
*	
*	Inputs:
*	unsigned char SlaveAddress - I2C address of the chip
*	short address - 16-bit memory address to read the data from
*	unsigned char *data - pointer to data string to be read
*	int length - length of data string to be read
*	
*	Description:
*	This function uses I2C communication to interface with the 
*	24AA1025 EEPROM.  A string of data (with length bytes) 
*	is read from the I2C device with SlaveAddress.  Data is read from
*	the specified memory address on the EEPROM using the function
*	MasterReadI2C1, which reads a string of data one byte at a time.
*	
*******************************************************************/
void receiveChar(unsigned char SlaveAddress, short address, unsigned char *data, int length)
{
	sendReceiveRequest(SlaveAddress, address);	

	// get data one byte at a time
	while(length-1)
	{
		*data = MasterReadI2C();
		IdleI2C();		//Wait to complete

		data++; // go to next memory address
		length--;

		// ACKSTAT is 0 when slave acknowledge. if 1 then slave has not acknowledge the data.
		if (I2CSTATbits.ACKSTAT)
			break;
	}
	
	// Set the last character to null as it will otherwise be random data from the EEPROM
	data--;
	*data = 0;
	
	StopI2C();
	IdleI2C();
}