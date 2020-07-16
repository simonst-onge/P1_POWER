/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    acl.c

  @Description
        This file groups the functions that implement the ACL library.
        The library implements basic functions to configure the accelerometer 
        and read the accelerometer values (raw values and g values).
        The library uses I2C functions provided by I2C library.
        Include the file as well as i2c.c, i2c.h and config.h in the project 
        when this library is needed.
 
  @Author
    Cristian Fatu 
    cristian.fatu@digilent.ro
*/
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include <xc.h>
#include <sys/attribs.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "i2c.h"
#include "accel.h"
#include "system_definitions.h"
#include <stdio.h>
#include "lcd.h"
#include "ssd.h"
#include "app_commands.h"

/* ************************************************************************** */
float fGRangeLSB;   // global variable used to pre-compute the value in g corresponding to each count of the raw value

static volatile int Flag_1s = 0;

volatile int Flag_rgb, Flag_acc = 0;
int reset = 0;
int compteur_flag_accel=0;
int per_rotat = 0;
int indexPaquet=0;
int e = 0;
int rpm = 0;
int tab[121];
// compteur d'échantillon total 
int cpt_ech = 0;
// compteur_rotations
int cpt_rot = 0;

uint8_t accel_buffer[accel_buf_length];
int32_t receive_buff[3][121];
//Valeur filtrées provenant de la Zybo
volatile signed int accXf, accYf, accZf; 

ACC_DATA accl;

/* ------------------------------------------------------------ */
/***	ACL_Init
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function initializes the hardware involved in the ACL module: 
**      the I2C1 module of PIC32 is configured to work at 400 khz, 
**      the ACL is initialized as #!?S!"//"CORRUPT!/"/"|]^| 
** 
**      Zut, le bloc de commentaires est corrompu, 
**      il faudra aller voir le datasheet pour vérifier/corriger la configuration.           
*/

void ACL_Init()
{
    
    ACL_ConfigurePins();
    I2C_Init(400000);
    
    ACL_GetRegister(ACL_CTRL_REG2);
    ACL_SetRegister(ACL_CTRL_REG2, 0x40);
    ACL_SetRegister(ACL_CTRL_REG3, 0x00);
    while(ACL_GetRegister(ACL_CTRL_REG2)&0x40);
    ACL_SetRegister(ACL_CTRL_REG4, 0x01); //interrupt enable
    ACL_SetRegister(ACL_CTRL_REG5, 0);
    ACL_GetRegister(ACL_INT_SOURCE);
    ACL_SetRegister(ACL_CTRL_REG1, 0x29); //12.5
   
    //pic32
    IFS0bits.INT4IF = 0; //flag initialisé à 0
    INTCONbits.INT4EP = 0; //change sur un rising edge
    IPC4bits.INT4IP=1;  //Priorité
    IEC0bits.INT4IE = 1; //enable interrupt
    
    accl.state = Init;
   
}

/* ------------------------------------------------------------ */
/***	ACL_ConfigurePins
**
**	Parameters:
**		
**
**	Return Value:
**		
**
**	Description:
**		This function configures the digital pins involved in the ACL module: 
**      the ACL_INT2 pin is configured as digital input.
**      The function uses pin related definitions from config.h file.
**      This is a low-level function called by ACL_Init(), so user should avoid calling it directly.
**          
*/
void ACL_ConfigurePins()
{
    // Configure ACL signals as digital input.
    tris_ACL_INT2 = 1;
}

void __ISR(_EXTERNAL_4_VECTOR, IPL2AUTO) ACL_ISR(void)
{
   Flag_1s = 1;            //indique à la boucle principale qu'on doit traiter
   Flag_rgb = 1;
   Flag_acc = 1; //indique à la boucle principale qu'on doit traiter
   IFS0bits.INT4IF = 0;     // clear interrupt flag
   
   ACL_ReadRawValues(accel_buffer);
   //SYS_CONSOLE_PRINT("accel\r\n");
}

uint16_t count= 0;
    unsigned char accel_state_machine_status = 0;
    unsigned char accel_state_machine_acknowledge = 0;
    unsigned char accel_state_machine_byte = 0;
    int accel_state_machine_cnt_timeout;

void accel_tasks()
{

    if (Flag_1s==1)
    {
        Flag_1s=0;
        LED4On();
    signed short accelX, accelY, accelZ; 
    
    accelX = ((signed int) accel_buffer[0]<<24)>>20  | accel_buffer[1] >> 4; //VR
    accelY = ((signed int) accel_buffer[2]<<24)>>20  | accel_buffer[3] >> 4; //VR
    accelZ = ((signed int) accel_buffer[4]<<24)>>20  | accel_buffer[5] >> 4; //VR

 

    if(accelX<0)
    {
        tab[compteur_flag_accel+1]= accelX |0xFFFFF000;
    }
    else 
    {
        tab[compteur_flag_accel+1]=accelX;
    }
    
    if(accelY<0)
    {
        tab[compteur_flag_accel+41]=accelY |0xFFFFF000;
    }
    
    else 
    {
        tab[compteur_flag_accel+41]=accelY;
    }
    if(accelZ<0)
    {
        tab[compteur_flag_accel+81]=accelZ |0xFFFFF000;
    }
    
    else 
    {
        tab[compteur_flag_accel+81]=accelZ;
    }
   
    
    SSD_WriteDigitsGrouped(count++, 0x1); 
    
    //if(SWITCH1StateGet()) //Pour afficher les donnees lorsquon joue avec la switch 1
    {
        //SYS_CONSOLE_PRINT("%d,%d,%d \r\n", accelX, accelY, accelZ);
        //SYS_CONSOLE_PRINT("inter : %d \r\n", IFS0bits.INT4IF);
    } 
    compteur_flag_accel++;
    //SYS_CONSOLE_PRINT("compteur: %d \r\n",compteur_flag_accel);
    if (compteur_flag_accel==40)
        {
            compteur_flag_accel=0;
            tab[0]=indexPaquet;
            indexPaquet++;
           
            UDP_Send_Packet = true; 
            //SYS_CONSOLE_PRINT("ready to send\r\n");        
            //SYS_CONSOLE_PRINT("%d \r\n",tab[1]);
    }
    }
}

void accel_rotation(void) //prend les valeurs qui arrive de la Zybo qui sont filtrées
{
//    int negatif = 0;
    if (Flag_acc == 1){
        Flag_acc = 0;
        cpt_ech ++;
        e ++;
    
    }
    if(UDP_Receive_Packet == true)
    {
       memcpy(receive_buff[2],receive_buff[1],484);
       memcpy(receive_buff[1],receive_buff[0],484);
       memcpy(receive_buff[0],UDP_Receive_Buffer,484);
       e = 0;

    }
    
    if (reset == 1){
        cpt_rot = 0;
        cpt_ech = 0;
        e = 0;
        accXf = 0;
        accYf = 0;
        accZf = 0;
//        accl.state = Init;
    }
    if(receive_buff[2][0] != 0){
        accXf = (signed int)receive_buff[2][e+1]; 
        accYf = (signed int)receive_buff[2][e+41];
        accZf = (signed int)receive_buff[2][e+81];
    }
    else{
        accXf = 0;
        accYf = 0;
        accZf = 0;
        cpt_ech = 0;
        e = 0;
    }
    
    if (e == 120){
        e = 0;
    }

    ///////////////////////////////////////////////////////////////////
    // sommation pour l'intégrale de parcours 
    int y = 0;
    int z = 0;
    int i = 0;
    int dy, dz = 400;
    int dyn, dzn = -400;
    for (i = 0; i < 40; i++){
        y += (signed int)receive_buff[2][i+41];
        z += (signed int)receive_buff[2][i+81];
    }
    if ((dyn <= y) && (y <= dy) && (dzn <= z) && (z <= dz)){
            cpt_rot++;
            // compteur d'échantillon total * période aquisition = temps 1 rotation
            per_rotat = cpt_ech * 10; 

            SYS_CONSOLE_PRINT("\r\n compteur de rotation \r\n");
            SYS_CONSOLE_PRINT("%d \r\n",cpt_rot);
//            rpm = 60000 / per_rotat;
//            SYS_CONSOLE_PRINT("\r\n rotation par minute \r\n");
//            SYS_CONSOLE_PRINT("%d \r\n ",rpm);
            cpt_ech = 0;
            y = 0;
            z = 0;
        }
    ///////////////////////////////////////////////////////////////////
    
//    switch (accl.state){
//        case Init:
//            
//            if (accXf > negatif && accYf > negatif) {
//                SYS_CONSOLE_PRINT("\r\n initialisation \r\n");
//                accl.state = QUA1;
//                
//            }
//            else {
//                accl.state = Init;
//            }
//           
//            break;
//        case QUA1:
//           
//            if (accXf < negatif && accYf > negatif) {
//                 SYS_CONSOLE_PRINT("\r\n Quadrant 1 \r\n");
//                 accl.state = QUA2;
//            }
//            else {
//                accl.state = QUA1;
//            }
//            
//            break;
//        case QUA2:
//            
//            if (accXf < negatif && accYf < negatif) {
//                SYS_CONSOLE_PRINT("\r\n Quadrant 2 \r\n");
//                 accl.state = QUA3;
//            }
//            else {
//                accl.state = QUA2;
//            }
//            
//            break;
//        case (QUA3) :
//            
//            if (accXf > negatif && accYf < negatif) {
//                SYS_CONSOLE_PRINT("\r\n Quadrant 3 \r\n");
//                 accl.state = QUA4;
//            }
//            else {
//                accl.state = QUA3;
//            }
//            
//            break;
//        case QUA4:
//            
//            if (accXf > negatif && accYf > negatif) {
//                SYS_CONSOLE_PRINT("\r\n Quadrant 4 \r\n");
//                accl.state = QUA1;
//                cpt_rot++;
//                // compteur d'échantillon total * période aquisition = temps 1 rotation
//                per_rotat = cpt_ech * 10; 
//                
//                SYS_CONSOLE_PRINT("\r\n compteur de rotation \r\n");
//                SYS_CONSOLE_PRINT("%d \r\n",cpt_rot);
//                rpm = 60000 / per_rotat;
//                SYS_CONSOLE_PRINT("\r\n rotation par minute \r\n");
//                SYS_CONSOLE_PRINT("%d \r\n ",rpm);
//                cpt_ech = 0;
//               
//            }
//            else {
//                accl.state = QUA4;
//            }
//            
//            break;
//    }
              
}


/* ------------------------------------------------------------ */
/***	ACL_SetRegister
**
**	Parameters:
**      unsigned char bAddress  - The register address.
**      unsigned char bValue    - The value to be written in the register.
**
**	Return Value:
**      unsigned char   0          success
**                      0xFF       the slave address was not acknowledged by the device.
**                      0xFE       timeout error
**
**	Description:
**		This function writes the specified value to the register specified by its address.
**      It returns the status of the operation: success or I2C errors (the slave address 
**      was not acknowledged by the device or timeout error).
**          
*/
unsigned char ACL_SetRegister(unsigned char bAddress, unsigned char bValue)
{
    unsigned char rgVals[2], bResult;
    rgVals[0] = bAddress;       // register address
    rgVals[1] = bValue;         // register value
    
    bResult = I2C_Write(ACL_I2C_ADDR, rgVals, 2, 1);

    return bResult;
}

/* ------------------------------------------------------------ */
/***	ACL_GetRegister
**
**	Parameters:
**      unsigned char bAddress  - The register address.
**
**	Return Value:
**      unsigned char   - The register value
**
**	Description:
**		This function returns the value of the register specified by its address. 
**      
**          
*/
unsigned char ACL_GetRegister(unsigned char bAddress)
{
    unsigned char bResult;
    if(I2C_Write(ACL_I2C_ADDR, &bAddress, 1, 0))return 0xFF;
    I2C_Read(ACL_I2C_ADDR, &bResult, 1);

    return bResult;
}

/* ------------------------------------------------------------ */
/***	ACL_GetDeviceID
**
**	Parameters:
**
**
**	Return Value:
**      unsigned char   device ID   The device ID obtained  by reading"0x0D: WHO_AM_I Device ID" register
**                      0xFF        the slave address was not acknowledged by the device.
**                      0xFE        timeout error
**
**	Description:
**		This function returns the device ID. It obtains it by reading 
**          "0x0D: WHO_AM_I Device ID" register.
**      If errors occur, it returns the I2C error 
**     (the slave address was not acknowledged by the device or timeout error)
**          
*/
unsigned char ACL_GetDeviceID()
{
    return ACL_GetRegister(ACL_DEVICE_ID);
}

/* ------------------------------------------------------------ */
/***	ACL_SetRange
**
**	Parameters:
**      unsigned char   bRange     - The full scale range identification
**                          0   +/-2g
**                          1   +/-4g
**                          2   +/-8g
**
**
**	Return Value:
**      unsigned char   0           success
**                      0xFF        the slave address was not acknowledged by the device.
**                      0xFE        timeout error
**
**	Description:
**		This function sets the full scale range. It sets the according bits in the 
**      0x0E: XYZ_DATA_CFG register. The function also pre-computes the fGRangeLSB 
**      to be used when converting raw values to g values.
**      It returns the status of the operation: success or I2C errors (the slave address 
**      was not acknowledged by the device or timeout error).
 */
unsigned char ACL_SetRange(unsigned char bRange)
{
    unsigned char bResult, bVal;
    bRange &= 3;    // only 2 least significant bits from bRange are used

    
    bVal = ACL_GetRegister(ACL_XYZDATACFG); // get old value of the register
    bVal &= 0xFC;   // mask out the 2 LSBs
    bVal |= bRange; // set the 2 LSBs according to the range value
    bResult = ACL_SetRegister(ACL_XYZDATACFG, bVal);

    // set fGRangeLSB according to the selected range
    unsigned char bValRange = 1<<(bRange + 2);
    fGRangeLSB = ((float)bValRange)/(1<<12);     // the range is divided to the resolution corresponding to number of bits (12)
    return bResult;
}

/* ------------------------------------------------------------ */
/***	ACL_ReadRawValues
**
**	Parameters:
**      unsigned char *rgRawVals     - Pointer to a buffer where the received bytes will be placed. 
**      It will contain the 6 bytes, one pair for each of to the 3 axes:
**                      rgRawVals[0]   - MSB of X reading (X11 X10 X9 X8 X7 X6 X5 X4)
**                      rgRawVals[1]   - LSB of X reading ( X3  X2 X1 X0  0  0  0  0)
**                      rgRawVals[2]   - MSB of Y reading (Y11 Y10 Y9 Y8 Y7 Y6 Y5 Y4)
**                      rgRawVals[3]   - LSB of Y reading ( Y3  Y2 Y1 Y0  0  0  0  0)
**                      rgRawVals[4]   - MSB of Z reading (Z11 Z10 Z9 Z8 Z7 Z6 Z5 Z4)
**                      rgRawVals[5]   - LSB of Z reading ( Z3  Z2 Z1 Z0  0  0  0  0)
**                                      In the above table, the raw value for each axis is a 12 bits value: 
**                                      X11-X0, Y11-Y0, Z11-Z0, the 0 bit being the LSB bit.
**
**	Return Value:
**
**	Description:
**		This function reads the module raw values for the three axes. 
**      Each raw value is represented on 12 bits, so it will be represented on 2 bytes:
**      The MSB byte contains the 8 MSB bits, while the LSB byte contains the 4 LSB bits, 
**      padded right with 4 bits of 0.
**      The raw values are obtained by reading the 6 consecutive registers starting with "0x01: OUT_X_MSB"
**      
*/
void ACL_ReadRawValues(unsigned char *rgRawVals)
{
    unsigned char bVal = ACL_OUT_X_MSB;

    I2C_Write(ACL_I2C_ADDR, &bVal, 1, 0);
    I2C_Read(ACL_I2C_ADDR, rgRawVals, 6);
}

/* ------------------------------------------------------------ */
/***	ACL_ConvertRawToValueG
**
**	Parameters:
**      unsigned char *rgRawVals     - Pointer to a buffer that contains the 2 bytes corresponding to the raw value. 
**                      rgRawVals[0]   - MSB of raw value (V11 V10 V9 V8 V7 V6 V5 V4)
**                      rgRawVals[1]   - LSB of raw value ( V3  V2 V1 V0  0  0  0  0)
**                                      In the above table, the raw value is a 12 bits value: V11-V0, the 0 bit being the LSB bit
**
**
**	Return Value:
**          float   the acceleration value in terms of g
**
**	Description:
**		This function returns the acceleration value in terms of g, computed from
**      the 2 bytes acceleration raw values (2's complement on 12 bites) 
**      Each raw value is represented on 12 bits so it will be represented on 2 bytes:
**      The MSB byte contains the 8 MSB bits of the raw value, while the LSB byte contains the 
**      4 LSB bits of the raw value padded right with 4 bits of 0.
**      Computing the acceleration in terms of g is done with this formula:
**      (<full scale range> / 2^12)*<raw value>. The (<full scale range> / 2^12) term is 
**      pre-computed every time the range is set, using global variable fGRangeLSB.
**      This function involves float values computing, so avoid using it intensively when performance is an issue.
**      
*/
float ACL_ConvertRawToValueG(unsigned char *rgRawVals)
{
    // Convert the accelerometer value to G's. 
    // With 12 bits measuring over a +/- ng range we can find how to convert by using the equation:
    // Gs = Measurement Value * (G-range/(2^12))

    unsigned short usReading = (((unsigned short)rgRawVals[0]) << 4) + (rgRawVals[1] >> 4);
    // extend sign from bit 12 bits to 16 bits
    if(usReading & (1<<11))
    {
        usReading |= 0xF000;
    }
  // fGRangeLSB is pre-computed in ACL_SetRange
    float fResult = ((float)((short)usReading)) * fGRangeLSB;
    return fResult;
}

/* ------------------------------------------------------------ */
/***	ACL_ReadGValues
**
**	Parameters:
**      float *rgGVals     - Pointer to a buffer where the acceleration values in terms of g will be placed.
**                          It will contain the 3 float values, one for each of to the 3 axes:
**                              0 - Acceleration on X axis (in terms of g) 
**                              1 - Acceleration on Y axis (in terms of g) 
**                              2 - Acceleration on Z axis (in terms of g) 
**
**
**	Return Value:
**
**	Description:
**		This function provides the acceleration values for the three axes, as float values in terms of g.
**      The raw values are acquired using ACL_ReadRawValues. Then, for each axis, the 2 bytes
**      are converted to a float value in terms of in g.
**      This function involves float values computing, so avoid using it intensively when performance is an issue.
**      
*/
void ACL_ReadGValues(float *rgGVals)
{
    unsigned char rgRawVals[6];
    ACL_ReadRawValues(rgRawVals);
    rgGVals[0] = ACL_ConvertRawToValueG(rgRawVals);
    rgGVals[1] = ACL_ConvertRawToValueG(rgRawVals + 2);
    rgGVals[2] = ACL_ConvertRawToValueG(rgRawVals + 4);
}


/* ------------------------------------------------------------ */
/***	ACL_Close
**
**	Parameters:
** 
**
**	Return Value:
**      
**
**	Description:
**		This functions releases the hardware involved in the ACL library: 
**      it closes the I2C1 interface.
**      
**          
*/
void ACL_Close()
{
    I2C1CONbits.ON = 0;     //Disable the I2C module 
}

/* *****************************************************************************
 End of File
 */
