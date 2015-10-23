
#include "stm32f30x.h"

//define PI
#define PI (3.141592654f)
//Multiply radians to get degrees.
#define toDegrees(x) (x * 57.2957795f)
//Multiply degrees to get radians.
#define toRadians(x) (x * 0.01745329252f)

// MPU9150 registers
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74

/********************************/

#define	MPUREG_SMPLRT_DIV					0x19	//0x07(125Hz)
#define	MPUREG_CONFIG							0x06	//0x06(5Hz) 0x1A(20Hz)
#define	MPUREG_GYRO_CONFIG					0x1B	//2000deg/s)
#define	MPUREG_ACCEL_CONFIG				0x1C	
#define	MPUREG_ACCEL_XOUT_H				0x3B
#define	MPUREG_ACCEL_XOUT_L				0x3C
#define	MPUREG_ACCEL_YOUT_H				0x3D
#define	MPUREG_ACCEL_YOUT_L				0x3E
#define	MPUREG_ACCEL_ZOUT_H				0x3F
#define	MPUREG_ACCEL_ZOUT_L				0x40
#define	MPUREG_TEMP_OUT_H					0x41
#define	MPUREG_TEMP_OUT_L					0x42
#define	MPUREG_GYRO_XOUT_H					0x43
#define	MPUREG_GYRO_XOUT_L					0x44	
#define	MPUREG_GYRO_YOUT_H					0x45
#define	MPUREG_GYRO_YOUT_L					0x46
#define	MPUREG_GYRO_ZOUT_H					0x47
#define	MPUREG_GYRO_ZOUT_L					0x48
#define	MPUREG_PWR_MGMT_1					0x6B	
#define	MPUREG_WHO_AM_I						0x75	


#define ACCEL_MGLSB         8192.0f  // 8192 LSB/mg +/-4g full scale range
#define GYRO_LSB_DPS        16.4f    // Least Significant Bits/Degrees/Second (+/-250deg/s range)
#define MAG_LSB_BPUT        0.315f    // Magnetic Sensitivity in Bits / micro-Tesa (uT)

#define I2C_EVENT_ERROR               (1 << 1)
#define I2C_EVENT_ERROR_NO_SLAVE      (1 << 2)
#define I2C_EVENT_TRANSFER_COMPLETE   (1 << 3)
#define I2C_EVENT_TRANSFER_EARLY_NACK (1 << 4)
#define I2C_EVENT_ALL                 (I2C_EVENT_ERROR |  I2C_EVENT_TRANSFER_COMPLETE | I2C_EVENT_ERROR_NO_SLAVE | I2C_EVENT_TRANSFER_EARLY_NACK)



void MPU9150_Init(void);
        
void MPU9150_Read_Acc(float *x, float *y, float *z);
void MPU9150_Read_Gyro(float *x, float *y, float *z);

void MPU9150_Read_Acc_xdkj(int *x, int *y, int *z);


int MPU9150_DataWrite(int address, char* data, int length, uint8_t repeated);
int MPU9150_DataRead(int address, char* data, int length, uint8_t repeated);







