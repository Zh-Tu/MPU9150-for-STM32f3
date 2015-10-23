
#include "stm32f30x.h"
#include "stm32f30x_i2c.h"
#include <stdio.h>
#include "MPU9150.h"

////////////////////////////////////////STRUCTURE DEFINE///////////////////////////////////////////////

#define FLAG_TIMEOUT ((int)0x4000)
#define LONG_TIMEOUT ((int)0x8000)
#define IMUAddress						0xd0
#define IMU_I2C								I2C1	

uint16_t Address_mpu6050 = 0xd0;
uint16_t MPU9150_addr = 0xd0;
uint16_t addr_mag = 0x18;
uint16_t MPU9150_mag_addr = 0x18;
////////////////////////////////////////FUNCTION DEFINE/////////////////////////////////////////////
uint8_t MPU9150_RegWrite(int addr_i2c,int addr_reg, char v);
int MPU9150_DataWrite(int address, char* data, int length, uint8_t repeated);
int I2C_Write(int address, char *data, int length, int stop);
int I2C_ByteWrite(int data);

uint8_t MPU9150_RegRead(int addr_i2c,int addr_reg, char *v);
int MPU9150_DataRead(int address, char* data, int length, uint8_t repeated);
int I2C_DataRead(int address, char *data, int length, int stop);
int I2C_ByteRead(int last);

////////////////////////////////////////Sensor fusion/////////////////////////////////////////////
void getEulerAngles(float angles[3], float RotMatrix[9]);
void quatern2rotMat(float q[4], float R[9]);
float invSqrt(float x);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);


void I2C_Stop(void);
extern void delay_ms(__IO uint32_t nCount);

void I2C_Congiguration(void)
{

  GPIO_InitTypeDef  GPIO_InitStructure; 
  I2C_InitTypeDef  I2C_InitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);

	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB , ENABLE); //GPIO clock enable
 
    
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9; ////last version PB9&PB8 as SDA&SCL which is I2C1 channel
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_4);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; //0x0000
  I2C_InitStructure.I2C_OwnAddress1 = IMUAddress; //boot I2C1_MPU9150 = 0xd0 which means Address is low
	I2C1->CR2 |= (0xd0 << 1); // 7-bit address format allow
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; // I2C_Ack_Enable = 0x0400
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // I2C_AcknowledgedAddress_7bit = 0x4000 , RESPONSE 1 BYTE
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_Timing = 400000; // i2c frequency 400000

  I2C_Init(I2C1, &I2C_InitStructure);
	
  I2C_Cmd(I2C1, ENABLE);

	I2C_AcknowledgeConfig(I2C1, ENABLE);    	
	
}


void MPU9150_Init(void)
{

	I2C_Congiguration();	
	delay_ms(100);
	MPU9150_RegWrite(Address_mpu6050,MPUREG_PWR_MGMT_1,0x80);
	delay_ms(10);
	MPU9150_RegWrite(Address_mpu6050,MPUREG_PWR_MGMT_1,0x03); //Select gyro Z axis for clock
	delay_ms(10);
	MPU9150_RegWrite(Address_mpu6050,MPUREG_SMPLRT_DIV, 9); //Sample rate=50Hz Fsample=1Khz/(19+1) = 50Hz set 0x00 achieve 8KHz
	delay_ms(10);
 	MPU9150_RegWrite(Address_mpu6050,MPUREG_CONFIG, 0x04); //FS & DLPF FS=2000º/s, DLPF = 20Hz LPF set 0x00 achieve Accel&Gyro max bandwidth 
	delay_ms(10);
	MPU9150_RegWrite(Address_mpu6050,MPUREG_GYRO_CONFIG, 0x18); //BITS_FS_2000DPS
	delay_ms(10);
	MPU9150_RegWrite(Address_mpu6050,MPUREG_ACCEL_CONFIG, 0x09);//Accel scale +/-4g (Full Scale 8192 LSB/mg) or 4096 LSB bits/g
	delay_ms(1000);

}

//////////////////////////////////////////I2C write///////////////////////////////////////////
int I2C_ByteWrite(int data) //////////byte write with timeout check
{ 
    int timeout;
    // Wait until the previous byte is transmitted
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET) {
        if ((timeout--) == 0) {
            return 0;
        }
    }		
    I2C1->TXDR = (uint8_t)data;
    return 1;
}

int I2C_Write(int address, char *data, int length, int stop) //////////I2C write with event check
{
    int timeout;
    int count;
		char byte_n;
    /* update CR2 register */
    I2C1->CR2 = (I2C1->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
               | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SoftEnd_Mode | (uint32_t)I2C_Generate_Start_Write);

    for (count = 0; count < length; count++) {
				byte_n = data[count];
        I2C_ByteWrite(byte_n);
    }
    // Wait transfer complete
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
		I2C_ClearFlag(I2C1, I2C_FLAG_TC);

    // If not repeated start, send stop.
    if (stop) {
			  I2C1->CR2 |= I2C_CR2_STOP;
        /* Wait until STOPF flag is set */
        timeout = FLAG_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET) {
            timeout--;
            if (timeout == 0) {
                return -1;
            }
        }
        /* Clear STOP Flag */
				I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
        
    }
    return count;
}

uint8_t MPU9150_RegWrite(int addr_i2c, int addr_reg, char v)
{
		char data[2];
    data[0] = addr_reg;
		data[1] = v; 
		I2C_ByteWrite(addr_i2c);
		I2C_ByteWrite(data[0]);
		I2C_ByteWrite(data[1]);
    return MPU9150_DataWrite(addr_i2c, data, 2, 0) == 0;
}

int MPU9150_DataWrite(int address, char* data, int length, uint8_t repeated) 
{
    int stop = (repeated) ? 0 : 1;
    int written = I2C_Write(address, data, length, stop);

    return length != written;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////I2C read//////////////////////////////////////////////////////
int I2C_ByteRead(int last)
{
    int timeout;
    // Wait until the byte is received
    timeout = FLAG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET) {
        if ((timeout--) == 0) {
            return -1;
        }
    }
    return (int)I2C1->RXDR;
}

int I2C_DataRead(int address, char *data, int length, int stop)
{  
    int timeout;
    int count;
    int value;
    /* update CR2 register */
    I2C1->CR2 = (I2C1->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
               | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SoftEnd_Mode | (uint32_t)I2C_Generate_Start_Read);

    // Read all bytes
    for (count = 0; count < length; count++) {
        value = I2C_ByteRead(0);
        data[count] = (char)value;
    }
    // Wait transfer complete
    timeout = LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
    I2C_ClearFlag(I2C1, I2C_FLAG_TC);
    // If not repeated start, send stop.
    if (stop) {
        I2C1->CR2 |= I2C_CR2_STOP;
        /* Wait until STOPF flag is set */
        timeout = FLAG_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET) {
            timeout--;
            if (timeout == 0) {
                return -1;
            }
        }
        /* Clear STOP Flag */
       I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
    }
    return length;
}

uint8_t MPU9150_RegRead(int addr_i2c,int addr_reg, char *v)
{
    char data = addr_reg; 
    uint8_t result = 0;

    if ((MPU9150_DataWrite(addr_i2c, &data, 1, 0) == 0) && (MPU9150_DataRead(addr_i2c, &data, 1, 0) == 0)){
        *v = data;
        result = 1;
    }
    return result;
}

int MPU9150_DataRead(int address, char* data, int length, uint8_t repeated) 
{
    int stop = (repeated) ? 0 : 1;
    int read =  I2C_DataRead(address, data, length, stop);
    return length != read;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_Stop(void)
{
    // Generate the STOP condition
    I2C1->CR2 |= I2C_CR2_STOP; 
}

///////////////////////////////////////////////////////////ÏÈ¶Ë¿Æ¼¼ÑÐ¾¿///////////////////////////////////////////////////////
/*
void MPU9150_Read_Acc_xdkj(int *x, int *y, int *z)
{
    signed int x_a, y_a, z_a;
    char c_data[6];  
		char data = MPUREG_ACCEL_XOUT_H;
		MPU9150_DataWrite(Address_mpu6050, &data, 1, 0); //ACCEL_XOUT_H reg starting address
		MPU9150_DataRead(Address_mpu6050, c_data, 6, 0);   
    x_a =  c_data[0] << 8;
    x_a += c_data[1];
    y_a =  c_data[2] << 8;
    y_a += c_data[3];
    z_a =  c_data[4] << 8;
    z_a += c_data[5];   
    //two's complement  
    if(x_a >= 0x8000)
        x_a -= 0xFFFF;
    if(y_a >= 0x8000)
        y_a -= 0xFFFF;
    if(z_a >= 0x8000)
        z_a -= 0xFFFF;      
		
		*x = x_a; ///////ÏÈ¶Ë¿Æ¼¼
		*y = y_a;
		*z = z_a;
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MPU9150_Read_Acc(float *x, float *y, float *z)
{
    signed int x_a, y_a, z_a;
    char c_data[6];  
		char data = MPUREG_ACCEL_XOUT_H;
		MPU9150_DataWrite(Address_mpu6050, &data, 1, 0); //ACCEL_XOUT_H reg starting address
		MPU9150_DataRead(Address_mpu6050, c_data, 6, 0);   
    x_a =  c_data[0] << 8;
    x_a += c_data[1];
    y_a =  c_data[2] << 8;
    y_a += c_data[3];
    z_a =  c_data[4] << 8;
    z_a += c_data[5];   
    //two's complement  
    if(x_a >= 0x8000)
        x_a -= 0xFFFF;
    if(y_a >= 0x8000)
        y_a -= 0xFFFF;
    if(z_a >= 0x8000)
        z_a -= 0xFFFF;      
    *x = (float)((float)x_a / ACCEL_MGLSB) * 9.801f;  //return m/s^2
    *y = (float)((float)y_a / ACCEL_MGLSB) * 9.801f;
    *z = (float)((float)z_a / ACCEL_MGLSB) * 9.801f;

}

void MPU9150_Read_Gyro(float *x, float *y, float *z)
{
    int x_g, y_g, z_g;
    char c_data[6];
		char data = MPUREG_GYRO_XOUT_H;
		MPU9150_DataWrite(Address_mpu6050, &data, 1, 0); //GYRO_XOUT_H reg starting address
		MPU9150_DataRead(Address_mpu6050, c_data, 6, 0);
    x_g =  c_data[0] << 8;
    x_g += c_data[1];
    y_g =  c_data[2] << 8;
    y_g += c_data[3];
    z_g =  c_data[4] << 8;
    z_g += c_data[5];
    //two's complement  
    if(x_g >= 0x8000)
        x_g -= 0xFFFF;
    if(y_g >= 0x8000)
        y_g -= 0xFFFF;
    if(z_g >= 0x8000)
        z_g -= 0xFFFF;
    //convert to measurement to degrees/sec, then to radians/sec
    *x = toRadians((float)((float)x_g / GYRO_LSB_DPS)); 
    *y = toRadians((float)((float)y_g / GYRO_LSB_DPS));
    *z = toRadians((float)((float)z_g / GYRO_LSB_DPS));
}




