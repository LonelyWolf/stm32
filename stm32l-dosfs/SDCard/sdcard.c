//*****************************************************************************
//
//! \file sdcard.c
//! \brief Driver for SDCard(Based on CoX Peripheral Interface).
//! \version V2.1.1.0
//! \date 11/15/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox 
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "xhw_types.h"
#include "xhw_memmap.h"
#include "xhw_ints.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xgpio.h"
#include "xspi.h"
#include "hw_sdcard.h"
#include "sdcard.h"


//
// 100ms correspond to SPI clock(unit: 8 clocks)
//
#define SD_READ_TIMEOUT_100MS   100 * SD_SPI_CLOCK / 1000 / 8

//
// 250ms correspond to SPI clock(unit: 8 clocks)
//
#define SD_WRITE_TIMEOUT_250MS  250 * SD_SPI_CLOCK / 1000 / 8 

//
// ACMD41 timeout value
//
#define SD_IDLE_WAIT_MAX        5000

//
//! SD Card Device Info
//
tSDCardDeviceInfo g_sSDCardInfo = 
{
    0,
    0,
    0,
    SD_READ_TIMEOUT_100MS,
    SD_WRITE_TIMEOUT_250MS,
    SD_WRITE_TIMEOUT_250MS,
    {0},        
};

//
//! Register CSD TAAC time unit
//! Unit = 0.000000001ns
//
const unsigned long g_pulCSDTAACTimeUnit[8] = 
{
//  1ns,        10ns,       100ns       1000ns(1us) 
    1000000000, 100000000,  10000000,   1000000,
//  10us        100us,      1ms,        10ms     
    100000,     10000,      1000,       100
};


//
//! Register CSD TAAC time value
//
const unsigned char g_pucCSDTAACTimeValue[16] = 
{
    0,  10, 12, 13, 15, 20, 25, 30,
    35, 40, 45, 50, 55, 60, 70, 80,    
};

//
// Register CSD R2W factor
//
const unsigned char g_pucCSDR2WFactor[8] = 
{
    1, 2, 4, 8, 16, 32, 0, 0    
};

//*****************************************************************************
//
// SD Card Hardware Abstraction Layer
// {
//
//*****************************************************************************
//
//! SPI CS Pin -> 0
//
#define SDSPICSAssert()                                                       \
        xGPIOSPinWrite(SD_PIN_CS, 0)

//
//! SPI CS Pin -> 1
//
#define SDSPICSDeAssert()                                                     \
        xGPIOSPinWrite(SD_PIN_CS, 1)

//
//! SPI Write/Send a byte(dump read)
//
#define SDSPIByteWrite(ucByte)                                                \
        xSPISingleDataReadWrite(SD_HOST_SPI_PORT, ucByte)

//
//! SPI Read/Receive a byte
//
#define SDSPIByteRead()                                                       \
        xSPISingleDataReadWrite(SD_HOST_SPI_PORT, 0xFF)

//
//! Set SPI Clock to normal after SD Card init
//
#define SDSPIClkToMax()                                                       \
        xSPIConfigSet(SD_HOST_SPI_PORT,                                       \
                      SD_SPI_CLOCK,                                           \
                      (xSPI_MOTO_FORMAT_MODE_0 | xSPI_MODE_MASTER |           \
                       xSPI_DATA_WIDTH8 | xSPI_MSB_FIRST))

//*****************************************************************************
//
// }
//
//*****************************************************************************

//
//! Get some bit-field from CSD register
//
#define SDCSDExtBits(pucCSDBuf, ulMSB, ulLSB)                                 \
        SDExtBits(pucCSDBuf, 16, ulMSB, ulLSB)

#if SD_CRC_EN
//
// CRC Table : X16 + X12 + X5 + 1
//
unsigned short const g_pusCRCTable[256] = 
{
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};
#endif

#if SD_CRC_EN
//*****************************************************************************
//
//! \brief Get the direction and mode of a pin.
//!
//! \param pucSource is the data to ge CRC16.
//! \param usLen is the data length.
//! 
//! \return Returns the CRC16 code.
//
//*****************************************************************************
unsigned short 
SDCRC16Get(unsigned char *pucSource, unsigned short usLen)
{
    unsigned short i;
    unsigned short ulResult = 0;

    for(i = 0; i < usLen; i++)
    {
        ulResult = (ulResult << 8) ^ 
            (unsigned short)g_pusCRCTable[(ulResult >> 8) ^ pucSource[i]];
    }
    
    return ulResult;
}

//*****************************************************************************
//
//! \brief get the CRC7 of the command of SD card.
//!
//! \param ucCmd is the SD Card command.
//! \param pucParam is the pucParam of command, length is 4 bytes.
//! 
//! \return Returns the CRC16 code.
//
//*****************************************************************************
unsigned char 
SDCmdByte6Get(unsigned char ucCmd, unsigned char *pucParam)
{
    unsigned char i, j;
    unsigned char ucReg = 0;
    unsigned char pucArray[5];
    
    pucArray[0] = ucCmd;
    
    //
    // Re-order the command
    //
    for(i = 1; i < 5; i++)
    {
        pucArray[i] = pucParam[4 - i];
    }
    
    //
    // Calc the CRC7
    //
    for(i = 0; i < 5; i++)
    {
        for(j = 0; j < 8; j++)
        {
            ucReg <<= 1;
            ucReg ^= ((((pucArray[i] << j) ^ ucReg) & 0x80) ? 0x9 : 0);
        }
    }
    
    //
    // CRC7 Result left shift 1, then set the bit 0(E) 
    //
    return ((ucReg << 1) & 0x01) ;    
}
#endif

//*****************************************************************************
//
//! \brief Initializate the hardware.
//!
//! \param None.
//!
//! This function initializate the hardware, such as the SPI port, re-power the 
//! sdcard.
//!
//! \return None.
//
//*****************************************************************************
void 
SDHardwareInit(void)
{
    //
    // Enable GPIO Port that used
    //    
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(SD_PIN_CS));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(SD_PIN_DATAIN));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(SD_PIN_CLK));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(SD_PIN_DATAOUT)); 
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(SD_PIN_POWER));
    
    //
    // Set Power Pin to GPIO Output and Power down the SD Card
    //
    xGPIOSPinTypeGPIOOutput(SD_PIN_POWER);
    xGPIOSPinWrite(SD_PIN_POWER, (SD_PIN_POWER_ACTIVE ? 0 : 1));
    
    //
    // Delay then Power on
    //
    xSysCtlDelay(0x9000);
    xGPIOSPinWrite(SD_PIN_POWER, SD_PIN_POWER_ACTIVE);
    
    //
    // Set CS Pin Type to GPIO Output
    //
    xGPIOSPinTypeGPIOOutput(SD_PIN_CS);
    xGPIOSPinWrite(SD_PIN_CS, 1);

    //
    // Set DataIn/CLK/DataOut Pin type to SPI
    //
    xSPinTypeSPI(SD_HOST_SPI_PIN_MOSI, SD_PIN_DATAIN);
    xSPinTypeSPI(SD_HOST_SPI_PIN_CLK, SD_PIN_CLK);
    xSPinTypeSPI(SD_HOST_SPI_PIN_MISO, SD_PIN_DATAOUT);

    //
    // Enable SPI 
    //
    xSysCtlPeripheralEnable2(SD_HOST_SPI_PORT);
    
    //
    // Set to 100kHz for initialisation
    //
    xSPIConfigSet(SD_HOST_SPI_PORT, 
                  100000, 
                  (xSPI_MOTO_FORMAT_MODE_0 | xSPI_MODE_MASTER | 
                   xSPI_DATA_WIDTH8 | xSPI_MSB_FIRST));
    xSPISSSet(SD_HOST_SPI_PORT, xSPI_SS_SOFTWARE, xSPI_SS_NONE);
    xSPIEnable(SD_HOST_SPI_PORT);
}

//*****************************************************************************
//
//! \brief Send a command to sdcard and get the response.
//!
//! \param ucCmd is the CMD index.
//! \param pucParam is the CMD parameter, 4 byte
//! \param ucRespType is the response type, such as R1.
//! \param pucResp is the response that command return.
//!
//! \b ucCmd can be SD_CMD1, SD_ACMD41 and so on, more refrence \ref SD_Command.\n
//! The means of \b pucParam is determined by the command, it is 4 bytes length.\n
//! \b ucRespType can be SD_R1, SD_R2 or SD_CMD1_R1, SD_ACMD41_R and so on, more
//! refrence \ref SD_Command and \ref SD_Response_Type. \n
//! \b pucResp is the response values, users can use the resonse to determine if
//! the command execute successful.
//!
//! \return None.
//
//*****************************************************************************
unsigned char SDCmdWrite(unsigned char ucCmd, unsigned char *pucParam,
                         unsigned char ucRespType, unsigned char* pucResp)
{
    unsigned char ucTmp;
    int i, iRdLen;
    
    xASSERT(pucParam);
    xASSERT(pucResp);
    
    SDSPICSAssert();
    
    //
    // send command header and word
    //
    SDSPIByteWrite((ucCmd & 0x3F) | 0x40);
    
    //
    // send parameters
    //
    for(i = 3; i >= 0; i--)
    {
       SDSPIByteWrite(pucParam[i]);
    }
    
    //
    // CRC
    //
#if SD_CRC_EN
    ucTmp = SDCmdByte6Get((ucCmd & 0x3F) | 0x40, pucParam);
    SDSPIByteWrite(ucTmp);
#else
    //
    // CRC,only used for the first command
    //
    SDSPIByteWrite(0x95);                                  
#endif
    
    //
    // Get the response byte length according the response type
    //  
    iRdLen = 0;
    switch(ucRespType)
    {
        case SD_R1:
        case SD_R1B:
        {
            iRdLen = 1;
            break;
        }
        case SD_R2:
        {
            iRdLen = 2;
            break;
        }
        case SD_R3:
        case SD_R7:
        {
            iRdLen = 5;
            break;
        }
        default:
        {
            SDSPIByteWrite(0xFF);
            SDSPICSDeAssert();
            return SD_ERR_CMD_RESP_TYPE;
            break;
        }
    }
    
    //
    // Waiting valid response
    //
    for(i = 0; i < SD_CMD_TIMEOUT; i++)
    {
        ucTmp = SDSPIByteRead();
        if(!(ucTmp & 0x80))
        {
            break;   
        } 
    }
    if(i >= SD_CMD_TIMEOUT)
    {
        SDSPIByteWrite(0xFF);
        SDSPICSDeAssert();
        return SD_ERR_CMD_TIMEOUT;        
    }
    
    //
    // Read the response
    //
    for(i = iRdLen - 1; i >= 0; i--)
    {
        pucResp[i] = ucTmp;
        ucTmp = SDSPIByteRead();
    }
    
    SDSPICSDeAssert();
    
    return SD_NO_ERR;
}

//*****************************************************************************
//
//! \brief Packet the command parameter into a byte buffer.
//!
//! \param ulValue is the parameter value.
//!
//! This function packet the unsigned long parameter value into a 4 byte buffer 
//! buffer.
//!
//! \return None.
//
//*****************************************************************************
void 
SDParamPack(unsigned char *pucParam, unsigned long ulValue)
{
    xASSERT(pucParam);
    
    pucParam[3] = (unsigned char)(ulValue >> 24);
    pucParam[2] = (unsigned char)(ulValue >> 16);
    pucParam[1] = (unsigned char)(ulValue >> 8);
    pucParam[0] = (unsigned char)(ulValue);
}

//*****************************************************************************
//
//! \brief Send a block command to sdcard.
//!
//! \param ucCmd is the block command index.
//! \param ucRespType is the command response type.
//! \param ulParameter is the command parameter.
//!
//! \b ucCmd block command can be SD_CMD24, SD_CMD16. more refrence 
//! \ref SD_Command_Block_Read, \ref SD_Command_Block_Write. \n
//! The \b ucRespType is always like this:  SD_CMD24's response type is 
//! SD_CMD24_R.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDBlockCommand(unsigned char ucCmd, unsigned char ucRespType, 
               unsigned long ulParameter)
{
    unsigned char pucParam[4],ucResp,ucRet;
    
    ulParameter <<= SD_BLOCK_SIZE_NBITS;

    SDParamPack(pucParam, ulParameter);

    ucRet = SDCmdWrite(ucCmd, pucParam, ucRespType, &ucResp);
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;
    }
    
    //
    // Check the R1 response
    //
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;
    }
         
    return SD_NO_ERR;
    
}

//*****************************************************************************
//
//! \brief Reset the sdcard(CMD0).
//!
//! \param None.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
static unsigned char 
SDCardReset(void)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp;
    
    return (SDCmdWrite(SD_CMD0, pucParam, SD_CMD0_R, &ucResp));
}

//*****************************************************************************
//
//! \brief Determine the SDCard is V1.x or V2.x.(CMD8)
//!
//! \param pbVoltageAccepted is the result of if 3V3 compatible.
//!
//! \return Returns SD_NO_ERR(0) indicate this is an V2.x sdcard, others V1.x or
//! not an sdcard.
//
//*****************************************************************************
static unsigned char
SDConditionCheck(xtBoolean *pbVoltageAccepted)
{
    //
    // 3V3
    //
    unsigned long ulParameter = (0x01 << 8) | 0xAA;
    unsigned char pucParam[4] = {0,0,0,0}, pucResp[5], ucTmp;  
    int i;
    
    xASSERT(pbVoltageAccepted);
    
    SDParamPack(pucParam, ulParameter);
    
    SDSPICSAssert();
    
    //
    // Send command header and word
    //
    SDSPIByteWrite((SD_CMD8 & 0x3F) | 0x40);
    
    //
    // Send parameters
    //
    for(i = 3; i >= 0; i--)
    {
       SDSPIByteWrite(pucParam[i]);
    }
    
    //
    // CMD8 is an SD mode command, need CRC
    //
    SDSPIByteWrite(0x87);
    
    //
    // Waiting response
    //
    for(i = 0; i < SD_CMD_TIMEOUT; i++)
    {
        ucTmp = SDSPIByteRead();
        if(!(ucTmp & 0x80))
        {
            break;   
        } 
    }
    if(i >= SD_CMD_TIMEOUT)
    {
        SDSPIByteWrite(0xFF);
        SDSPICSDeAssert();
        return SD_ERR_CMD_TIMEOUT;        
    }
    
    //
    // Read the response
    //
    for(i = 5 - 1; i >= 0; i--)
    {
        pucResp[i] = ucTmp;
        ucTmp = SDSPIByteRead();
    }
    
    SDSPICSDeAssert();
    
    //
    // Check the resonse, first byte is R1
    //
    if(pucResp[0] != 0)
    {
        return SD_ERR_CMD_RESP;   
    }
    
    // support 3V3 ?
    if((pucResp[4] & SD_R7_4_VOLTAGE_MASK) == 0x01)
    {
        *pbVoltageAccepted = xtrue;
    }
    else
    {
        *pbVoltageAccepted = xfalse;        
    }
    
    return SD_NO_ERR;
    
}

//*****************************************************************************
//
//! \brief Read the CSD Register.
//!
//! \param ucCSDLen is the CSD register length, the full register is 16 byte.
//! \param pucRdBuf is the destination buffer to store the register value.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDCSDRead(unsigned char ucCSDLen, unsigned char *pucRdBuf)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp, ucRet;
  
    ucRet = SDCmdWrite(SD_CMD9, pucParam, SD_CMD9_R, &ucResp);
    if(ucRet != SD_NO_ERR)                                     
    {
        return ucRet;                                    
    }
  
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;
    }
    
    return (SDRegisterRead(ucCSDLen, pucRdBuf));
}

//*****************************************************************************
//
//! \brief Read the CID Register.
//!
//! \param ucCIDLen is the CID register length, the full register is 16 byte.
//! \param pucRdBuf is the destination buffer to store the register value.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDCIDRead(unsigned char ucCIDLen, unsigned char *pucRdBuf)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp, ucRet;
  
    ucRet = SDCmdWrite(SD_CMD10, pucParam, SD_CMD10_R, &ucResp);
    if(ucRet != SD_NO_ERR)                                     
    {
        return ucRet;                                    
    }
  
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;
    }
    
    return (SDRegisterRead(ucCIDLen, pucRdBuf));
}

//*****************************************************************************
//
//! \brief Forces the card to stop transmission in Multiple Block Read Operation.
//!
//! \param None.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDTransmissionStop(void)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp;
    
    return (SDCmdWrite(SD_CMD12, pucParam, SD_CMD12_R, &ucResp));
}

//*****************************************************************************
//
//! \brief Read the status Register.
//!
//! \param ucLen is the status register length.
//! \param pucRdBuf is the destination buffer to store the register value.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDCardStatusRead(unsigned char ucLen, unsigned char *pucBuffer)
{
    unsigned char pucParam[4] = {0,0,0,0};
    
    return (SDCmdWrite(SD_CMD13, pucParam, SD_CMD13_R, pucBuffer));
}

//*****************************************************************************
//
//! \brief Set the block size.
//!
//! \param ulLength is the block size, the size is always 512 bytes.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDBlockLenSet(unsigned long ulLength)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp, ucRet;
    
    SDParamPack(pucParam, ulLength);
    ucRet = SDCmdWrite(SD_CMD16, pucParam, SD_CMD16_R, &ucResp);
    
    if(ucRet !=SD_NO_ERR)
    {
        return ucRet;
    }
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;
    }
    
    return SD_NO_ERR;
}

//*****************************************************************************
//
//! \brief Send request of reading a block.
//!
//! \param ulBlockAddr the block address.
//!
//! The parameter \b ulBlockAddr, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDSingleBlockRead(unsigned long ulBlockAddr)
{
    return (SDBlockCommand(SD_CMD17, SD_CMD17_R, ulBlockAddr)); 
}

//*****************************************************************************
//
//! \brief Send request of reading multiple blocks.
//!
//! \param ulBlockAddr the start block address.
//!
//! The parameter \b ulBlockAddr, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDMultipleBlockRead(unsigned long ulBlockAddr)
{
    return (SDBlockCommand(SD_CMD18, SD_CMD18_R, ulBlockAddr)); 
}

//*****************************************************************************
//
//! \brief Send request to write a block.
//!
//! \param ulBlockAddr the block address.
//!
//! The parameter \b ulBlockAddr, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDSingleBlockWrite(unsigned long ulBlockAddr)
{
    return (SDBlockCommand(SD_CMD24, SD_CMD24_R, ulBlockAddr));
}

//*****************************************************************************
//
//! \brief Send request to write multiple blocks.
//!
//! \param ulBlockAddr the start block address.
//!
//! The parameter \b ulBlockAddr, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDMultipleBlockWrite(unsigned long ulBlockAddr)
{
    return (SDBlockCommand(SD_CMD25, SD_CMD25_R, ulBlockAddr));
}

#if SD_ERASE_BLOCK_EN
//*****************************************************************************
//
//! \brief Set the start block to erase.
//!
//! \param ulStartBlock the start block address.
//!
//! The parameter \b ulStartBlock, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDEraseStartBlockSet(unsigned long ulStartBlock)
{
    return (SDBlockCommand(SD_CMD32, SD_CMD32_R, ulStartBlock));
}

//*****************************************************************************
//
//! \brief Set the end block to erase.
//!
//! \param ulEndBlock the start block address.
//!
//! The parameter \b ulEndBlock, SDSC Card uses byte unit address and SDHC and 
//! SDXC Cards  use block unit address (512 bytes unit).
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDEraseEndBlockSet(unsigned long ulEndBlock)
{
    return (SDBlockCommand(SD_CMD33, SD_CMD33_R, ulEndBlock));
}

//*****************************************************************************
//
//! \brief Execute to erase the select blocks. 
//!
//! \param ulEndBlock the start block address.
//!
//! The start block is set by SDEraseStartBlockSet(), the end block is set by
//! SDEraseEndBlockSet().
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDEraseSelected(void)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucResp, ucRet;
    
    SDParamPack(pucParam, 0);
    ucRet = SDCmdWrite(SD_CMD38, pucParam, SD_CMD38_R, &ucResp);
    
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
    
    if(SDWaitBusy(SD_WAIT_ERASE) != SD_NO_ERR)
    {
        return SD_ERR_TIMEOUT_ERASE;
    }
    else
    {
        return SD_NO_ERR;
    } 
}
#endif

//*****************************************************************************
//
//! \brief Read the OCR Register.
//!
//! \param ucOCRLen is the OCR register length, the full register is 4 byte.
//! \param pucRdBuf is the destination buffer to store the register value.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDOCRRead(unsigned char ucOCRLen, unsigned char *pucRdBuf)
{
    unsigned char pucParam[4] = {0,0,0,0}, pucResp[5], ucTmp;

    ucTmp = SDCmdWrite(SD_CMD58, pucParam, SD_CMD58_R, pucResp);
    if(ucTmp != SD_NO_ERR)
    {
        return ucTmp;                                                 
    }
                                                    
    if(pucResp[0] != 0)
    {
        return SD_ERR_CMD_RESP;
    }
    
    for(ucTmp = 0; ucTmp < ucOCRLen; ucTmp++)
    {
        pucRdBuf[ucTmp] = pucResp[ucTmp + 1];
    }
    
    return SD_NO_ERR;
}

#if SD_CRC_EN
//*****************************************************************************
//
//! \brief Enable/Disable the CRC in SPI mode.
//!
//! \param bEnable is CRC enable state. xture means enable, xfalse means disable.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SD_EnableCRC(xtBoolean bEnable)
{
    unsigned char pucParam[4],ucResp,ucRet;
        
    if(bEnable)
    {
        pucParam[0] = 1;
    }
    else
    {
        pucParam[1] = 0;
    }
    ucRet = SDCmdWrite(SD_CMD59, pucParam, SD_CMD59_R, &ucResp);
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;                             
    }
    
    if(ucResp != 0)    
    {
        return SD_ERR_CMD_RESP;
    }
     
    return SD_NO_ERR;

}
#endif

#if SD_WRITE_MULTI_BLOCK_EN
//*****************************************************************************
//
//! \brief Get the blocks number that have been sucessful written with mulit 
//! block command.
//!
//! \param pulBlockNum is buffer to store the number.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDWrBlockNumberGet(unsigned long *pulBlockNum)
{
    unsigned char pucTmp[4] = {0,0,0,0},ucResp,ucRet;
  
    ucRet = SDCmdWrite(SD_CMD55, pucTmp, SD_CMD55_R, &ucResp);
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;
    }
         
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;        
    }
                                             
    ucRet = SDCmdWrite(SD_ACMD22, pucTmp, SD_ACMD22_R, &ucResp); 
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;                                                
    }
                                                                    
    if(ucResp != 0)
    {
        return SD_ERR_CMD_RESP;
    }
            
    ucRet = SDBlockDataRead(pucTmp, 4);
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;
    }
        
    *pulBlockNum = (pucTmp[0] << 24) + (pucTmp[1] << 16) + 
                   (pucTmp[2] << 8) + pucTmp[3];    

    return SD_NO_ERR;
}
#endif


///////////////////////////////////////////////////////////////////////////////


//*****************************************************************************
//
//! \brief Transfer the register data after sending register read request.
//!
//! \param ulLen is the length to transfer.
//! \param pucRdBuf is the destination buffer to store the value.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDRegisterRead(unsigned long ulLen, unsigned char* pucRdBuf)
{
    unsigned long i = 0;
    unsigned char ucResp;
    
    xASSERT(pucRdBuf);
    
    SDSPICSAssert();
    
    // wait for data start token
    i = 0;
    do
    {
        ucResp = SDSPIByteRead();
        i++;
    }while((ucResp == 0xFF) && (i < SD_READREG_TIMEOUT)); 
    
    if(i >= SD_READREG_TIMEOUT)
    {
        SDSPIByteWrite(0xFF);
        SDSPICSDeAssert();
        return SD_ERR_TIMEOUT_READ;
    }
    
    if(ucResp != SD_TOK_RD_START_BLOCK)
    {
        pucRdBuf[0] = ucResp;
        i = 1;
    }
    else
    {
        i = 0;   
    }
    
    for(; i < ulLen; i++)
    {
        pucRdBuf[i] = SDSPIByteRead();
    }
    
    // Read CRC16
    i = SDSPIByteRead();
    i = (i << 16) | SDSPIByteRead();
    
#if SD_CRC_EN 
    if(i != SDCRC16Get(pucRdBuf, ulLen))
    {
        SDSPICSDeAssert();
        SDSPIByteWrite(0xFF);                            
        return SD_ERR_DATA_CRC16;
    }    
#endif    
    
    SDSPIByteWrite(0xFF); 
    SDSPICSDeAssert();

    return SD_NO_ERR;      
}

//*****************************************************************************
//
//! \brief Transfer the block data after sending block read request.
//!
//! \param pucRdBuf is the destination buffer to store the value.
//! \param ulLen is the length to transfer.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDBlockDataRead(unsigned char *pucRdBuf, unsigned long ulLen)
{
    unsigned char ucTmp;
    unsigned long i = 0;
    
    SDSPICSAssert();
   
    do
    {
        ucTmp = SDSPIByteRead();
        i++;
    }while((ucTmp == 0xFF) && (i < g_sSDCardInfo.ulReadTimeout));

    
    if(i >= g_sSDCardInfo.ulReadTimeout)
    {
        SDSPICSDeAssert();
        return SD_ERR_TIMEOUT_READ;
    }
    
    if(ucTmp != SD_TOK_RD_START_BLOCK)
    {
        SDSPIByteWrite(0xFF);
        SDSPICSDeAssert();
        return SD_ERR_DATA_START_TOK;
    }
    
    for(i = 0; i < ulLen; i++)
    {
        pucRdBuf[i] = SDSPIByteRead();
    }
           
    i = SDSPIByteRead();                                
    i = (i << 8) + SDSPIByteRead();

#if SD_CRC_EN 
    if(i != SDCRC16Get(pucRdBuf, ulLen))
    {                                    
        SDSPIByteWrite(0xFF); 
        SDSPICSDeAssert();                          
        return SD_ERR_DATA_CRC16;    
    }    
#endif  

    SDSPIByteWrite(0xFF); 
    SDSPICSDeAssert();

    return SD_NO_ERR;
}

//*****************************************************************************
//
//! \brief Transfer the block data after sending block write request.
//!
//! \param pucWrBuf is the source buffer to transfer.
//! \param ulLen is the length to transfer.
//! \param bMulti indicate if this is an multi block write request.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDBlockDataWrite(unsigned char *pucWrBuf, unsigned long ulLen, 
                 unsigned char bMulti)
{
    unsigned short i;
    unsigned char ucTmp;

    SDSPICSAssert();
        
    SDSPIByteWrite(0xFF);
    
    if(bMulti)
    {
        SDSPIByteWrite(SD_TOK_WR_START_BLOCK_MULTI);
    }
    else
    {
        SDSPIByteWrite(SD_TOK_WR_START_BLOCK);
    }

    for(i = 0; i < ulLen; i++)
    {
        SDSPIByteWrite(pucWrBuf[i]);
    }

#if SD_CRC_EN    
    i = SDCRC16Get(pucWrBuf,ulLen);
#endif

    SDSPIByteWrite((i >> 8) & 0xFF);
    SDSPIByteWrite(i & 0xFF);
                
    ucTmp = SDSPIByteRead();
    if((ucTmp & SD_RESP_DATA_MASK) != SD_RESP_DATA_ACCETPTED)    
    {        
        SDSPIByteWrite(0xFF);
        SDSPICSDeAssert();
        return SD_ERR_DATA_RESP;
    }
        
    SDSPICSDeAssert();
             
    if(SDWaitBusy(SD_WAIT_WRITE) != SD_NO_ERR)            
    {
        return SD_ERR_TIMEOUT_WRITE;
    }
    else
    {
        return SD_NO_ERR;
    }
 }

//*****************************************************************************
//
//! \brief Send the token to stop multi block transfer.
//!
//! \param None.
//!
//! \return None.
//
//*****************************************************************************
void 
SDStopMultiToken(void)
{
    SDSPICSAssert();
    
    SDSPIByteWrite(0xFF);
    SDSPIByteWrite(SD_TOK_STOP_TRAN_MULTI);
    SDSPIByteRead();
    
    SDSPICSDeAssert();
}

//*****************************************************************************
//
//! \brief Wait the sd card read/write/erase execute complete.
//!
//! \param ucWaitType is the execute type to wait.
//! 
//! The parameter \b ucWaitType can be:
//! - \ref SD_WAIT_READ - wait read complete.
//! - \ref SD_WAIT_WRITE - wait write complete.
//! - \ref SD_WAIT_ERASE - wait erase complete.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDWaitBusy(unsigned char ucWaitType)
{
    unsigned long ulTimeout = 0, i = 0;
    unsigned char ucTmp;
    
    if(ucWaitType == SD_WAIT_WRITE)
    {
        ulTimeout = g_sSDCardInfo.ulWriteTimeout;
    }
    else if(ucWaitType == SD_WAIT_WRITE)
    {
        ulTimeout = g_sSDCardInfo.ulReadTimeout;   
    }
    else
    {
        ulTimeout = g_sSDCardInfo.ulEraseTimeout;
    }

    SDSPICSAssert();
    do
    {
        ucTmp = SDSPIByteRead();
        i++;
    }while ((ucTmp != 0xFF) && (i < ulTimeout));
    SDSPICSDeAssert();
    
    if(i < ulTimeout) 
    {
        return SD_NO_ERR;
    }
    else 
    {
        return SD_ERR_TIMEOUT_WAIT;
    }
}

//*****************************************************************************
//
//! \brief Delay some SPI clocks.
//!
//! \param None.
//!
//! The delay unit is 8 SPi clocks. So the total clocks delay is:
//! ulDelay8Clock * 8
//! 
//! \return None.
//
//*****************************************************************************
void 
SDSPIDelay(unsigned long ulDelay8Clock)
{
    unsigned long i;

    for(i = 0; i < ulDelay8Clock; i++)
    {
        SDSPIByteWrite(0xFF);
    }
}

//*****************************************************************************
//
//! \brief Calculate sdcard read/write/erase timeout according CSD register.
//!
//! \param pucCSDBuf is the CSD register buffer.(16 byte)
//!
//! \return None.
//
//*****************************************************************************
static void
SDCalTimeout(unsigned char *pucCSDBuf)
{
    unsigned long ulTmp;
    unsigned char ucTimeUnit,ucTimeValue,ucTimeoutFactor;
    
    xASSERT(pucCSDBuf);
    
    //
    // Set default timeout value
    //
    g_sSDCardInfo.ulReadTimeout = SD_READ_TIMEOUT_100MS;
    g_sSDCardInfo.ulWriteTimeout = SD_WRITE_TIMEOUT_250MS;
    g_sSDCardInfo.ulEraseTimeout = SD_WRITE_TIMEOUT_250MS;

    //
    // TAAC[119:112] (time unit [2:0], time value [6:3])
    // R2W Factor [28:26]
    //
    ucTimeUnit = SDCSDExtBits(pucCSDBuf, 119, 112) & 0x07;
    ucTimeValue = (SDCSDExtBits(pucCSDBuf, 119, 112) >> 3) & 0x0F;
    ucTimeoutFactor = SDCSDExtBits(pucCSDBuf, 28, 26);
    
    if(ucTimeValue == 0)    
    {
        return;
    }
    if(ucTimeoutFactor >= 6)
    {
        return;
    }
    
    ulTmp = SD_SPI_CLOCK * g_pucCSDTAACTimeValue[ucTimeValue] / 10 / 
            g_pulCSDTAACTimeUnit[ucTimeUnit];
    
    //
    // NSAC [114:104]
    //
    ulTmp += SDCSDExtBits(pucCSDBuf, 114, 104) * 100;
    
    //
    // 8 SPI Clock is a unit
    //
    g_sSDCardInfo.ulReadTimeout = ulTmp / 8;
    g_sSDCardInfo.ulWriteTimeout = (ulTmp * g_pucCSDR2WFactor[ucTimeoutFactor]) / 8;
        
    if(g_sSDCardInfo.ulReadTimeout > SD_READ_TIMEOUT_100MS)
    {
        g_sSDCardInfo.ulReadTimeout = SD_READ_TIMEOUT_100MS;
    }
    
    if(g_sSDCardInfo.ulWriteTimeout > SD_WRITE_TIMEOUT_250MS)
    {
        g_sSDCardInfo.ulWriteTimeout = SD_WRITE_TIMEOUT_250MS;
    }

    g_sSDCardInfo.ulEraseTimeout = g_sSDCardInfo.ulWriteTimeout;
}

//*****************************************************************************
//
//! \brief Parse sdcard info such as page size, block size, block number 
//! according the CSD register.
//!
//! \param None.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
static unsigned char
SDCardInfoParse(void)
{
    unsigned char ucRet;
    unsigned char pucCSDBuf[16];
    unsigned long ulMaxRdBlockSize = 0;
    
    //
    // Read CID register
    ucRet = SDCSDRead(16, g_sSDCardInfo.pucCID);
    if(ucRet != SD_NO_ERR)    
    {
        return ucRet;    
    }
    
    //
    //
    // Read CSD register 
    //
    ucRet = SDCSDRead(16, pucCSDBuf);
    if(ucRet != SD_NO_ERR)    
    {
        return ucRet;    
    }
    
    //
    // Calculate sdcard timeout according CSD
    //
    SDCalTimeout(pucCSDBuf);
    
    //
    // Calculate the size of a sector
    // READ_BL_LEN [83:80]
    // BlockSize = 2^READ_BL_LEN
    //
    ulMaxRdBlockSize = 1 << (SDCSDExtBits(pucCSDBuf, 83, 80));
    
    //
    // calculate the sector numbers of the SD Card
    // C_SIZE [73:62], C_SIZE_MULT[49:47]
    // BlockNumber = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2)
    //
    g_sSDCardInfo.ulBlockNumber = (SDCSDExtBits(pucCSDBuf, 73, 62) + 1) *
                                  (1 << (SDCSDExtBits(pucCSDBuf, 49, 47) + 2));
    
    //
    // Block Unit -> SD_BLOCK_SIZE(512)
    //
    g_sSDCardInfo.ulBlockNumber *=  (ulMaxRdBlockSize / SD_BLOCK_SIZE);
    
    //
    // calculate the size of sector(erase unit is block number)
    // SECTOR_SIZE[45:39]
    // Erase Unit = SECTOR_SIZE + 1
    //
    g_sSDCardInfo.ulEraseUnit = SDCSDExtBits(pucCSDBuf, 45, 39) + 1;
    
    //
    // Block Unit -> SD_BLOCK_SIZE(512)
    //
    g_sSDCardInfo.ulEraseUnit *= (ulMaxRdBlockSize / SD_BLOCK_SIZE);
                    
    return SD_NO_ERR;       
}

//*****************************************************************************
//
//! \brief Waiting sdcard internal initialization complete.
//!
//! \param None.
//!
//! Waiting sdcard init OK by repeating sending CMD1.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
static unsigned char 
SDActiveInit(void)
{
    unsigned char pucParam[4] = {0,0,0,0}, ucRet, ucResp;
    unsigned long i = 0;

    //
    // Wait unit the SD card initialize complete.
    //
    do 
    {
        //
        // The next command is an application specific command
        //
        ucRet = SDCmdWrite(SD_CMD55, pucParam, SD_CMD55_R, &ucResp);
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        
        ucRet = SDCmdWrite(SD_ACMD41, pucParam, SD_ACMD41_R, &ucResp);
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        i++;
    }while (((ucResp & 0x01) == 0x01) && (i < SD_IDLE_WAIT_MAX));
                                                        
    if(i >= SD_IDLE_WAIT_MAX)
    {
        return SD_ERR_TIMEOUT_WAIT_IDLE;
    }

    return SD_NO_ERR;
}

//*****************************************************************************
//
//! \brief Initializate the sdcard.
//!
//! \param None.
//!
//! This function initializate the hardware, re-power on the sdcard, do some 
//! initialization sequence, then get some sdcard info, such as block size,
//! block number, the CID and so on.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char 
SDInit(void)
{
    unsigned char pucRdBuf[4], ucRet;
    xtBoolean bVoltageAccepted = xfalse;
    
    //
    // Hardware Init(SPI, Power)
    //
    SDHardwareInit();
    
    //
    // Assert CS at least 74 SPI clocks
    //
    SDSPICSAssert();    
    SDSPIDelay(10);        
    SDSPICSDeAssert();    
    SDSPIDelay(1);        
    
    //
    // Reset sdcard
    //
    ucRet = SDCardReset();
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;                                    
    }
    
    ucRet = SDConditionCheck(&bVoltageAccepted);
    if(ucRet != SD_NO_ERR)
    {
        // Ver 1.x
        //
        // Read OCR register to check if 3V3 compatible
        //
        ucRet = SDOCRRead(4, pucRdBuf); 
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        //
        //3.2V - 3.4V [21:20]
        //
        if(!SDExtBits(pucRdBuf, 4, 21, 20))
        {
            //
            // Not support 3.3V
            //
            return SD_ERR_VOL_NOT_SUSP;   
        }        

        //
        // Waiting sdcard internal initialization complete
        //
        ucRet = SDActiveInit();
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        
        //
        // Ver1.X Standard Capacity SD Memory Card
        //
        g_sSDCardInfo.ulCardType = SD_TYPE_SDSC_V1;
        
    }
    else
    {
        // Ver 2.x
        if(!bVoltageAccepted)
        {
            return SD_ERR_VOL_NOT_SUSP;   
        }
        //
        // Read OCR register to check if 3V3 compatible
        //
        ucRet = SDOCRRead(4, pucRdBuf); 
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        //
        //3.2V - 3.4V [21:20]
        //
        if(!SDExtBits(pucRdBuf, 4, 21, 20))
        {
            //
            // Not support 3.3V
            //
            return SD_ERR_VOL_NOT_SUSP;   
        }

        //
        // Waiting sdcard internal initialization complete
        //
        ucRet = SDActiveInit();
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        //
        // Read OCR register to check CCS
        //
        ucRet = SDOCRRead(4, pucRdBuf); 
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        
        if(SDExtBits(pucRdBuf, 4, 30, 30) == 0)      
        {
            //
            // Ver2.X Standard Capacity SD Memory Card
            //
            g_sSDCardInfo.ulCardType = SD_TYPE_SDSC_V2;            
        }
        else
        {
            //
            // Ver2.X High Capacity or Extended Capacity SD Memory Card
            //
            g_sSDCardInfo.ulCardType = SD_TYPE_SDSC_V1;            
        }
    }
    
    //
    // Set SPI Clock to Max
    //
    SDSPIClkToMax();
    
#if SD_CRC_EN    
    //
    // Enable CRC
    //
    ucRet = SD_EnableCRC();
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
#endif
    
    //
    // Set the Block Length to 512 Byte
    //
    ucRet = SDBlockLenSet(SD_BLOCK_SIZE);
    if(ucRet != SD_NO_ERR)  
    {
        return ucRet;
    }
    
    //
    // Get CID, CSD register, then Parse
    //    
    return (SDCardInfoParse());
    
}

//*****************************************************************************
//
//! \brief Get some bit-field from a byte buffer.
//!
//! \param None.
//!
//! This function is always used to get a register bit field.
//!
//! \return Returns the bit-field value.
//
//*****************************************************************************
unsigned long
SDExtBits(unsigned char *pucBuf, unsigned long ulBufLen, 
          unsigned long ulMSB, unsigned long ulLSB)
{
    unsigned long i;
    unsigned long ulBits = 0, ulPosition, ulByte, ulBit, ulValue;
    unsigned long ulSize = 1 + ulMSB - ulLSB; 

    xASSERT(pucBuf);
    xASSERT(ulBufLen > 0);
    xASSERT(ulMSB >= ulLSB);
    xASSERT(ulMSB < (ulBufLen * 8));
    xASSERT(ulMSB - ulLSB < 32);
    
    for(i=0; i < ulSize; i++) 
    {
        ulPosition = ulLSB + i;
        ulByte = (ulBufLen - 1) - (ulPosition >> 3);
        ulBit = ulPosition & 0x7;
        ulValue = (pucBuf[ulByte] >> ulBit) & 1;
        ulBits |= ulValue << i;
    }
    return ulBits;
}

//*****************************************************************************
//
//! \brief Read a block from the sdcard.
//!
//! \param pucDestBuf is the destination buffer to store the value that read.
//! \param ulBlockIndex is the block index to read. The index start from 0.
//!
//! This function is used to read a block data from the sdcard, a block length 
//! is always 512 bytes, defined by SD_BLOCK_SIZE.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDBlockRead(unsigned char *pucDestBuf, unsigned long ulBlockIndex)
{
    unsigned char ucRet;
    xASSERT(pucDestBuf);

    //
    // Over the card range
    //    
    if(ulBlockIndex >= g_sSDCardInfo.ulBlockNumber)
    {
        return SD_ERR_OVER_CARD_RANGE;   
    }
    
    //
    // Set Read address
    //
    if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)
    {
        ucRet = SDSingleBlockRead(ulBlockIndex);
    }
    else
    {
        ucRet = SDSingleBlockRead(ulBlockIndex * SD_BLOCK_SIZE);   
    }
    
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
    
    //
    // Read Data
    //
    ucRet = SDBlockDataRead(pucDestBuf, SD_BLOCK_SIZE);
    
    return ucRet;    
}
#if SD_READ_MULTI_BLOCK_EN
//*****************************************************************************
//
//! \brief Read multi blocks from the sdcard.
//!
//! \param pucDestBuf is the destination buffer to store the value that read.
//! \param ulBlockIndex is the block index to read. The index start from 0.
//! \param ulRdBlockNumber is the number of blocks to read.
//!
//! This function is used to read some blocks data from the sdcard, a block  
//! length is always 512 bytes, defined by SD_BLOCK_SIZE.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDMultiBlockRead(unsigned char *pucDestBuf, unsigned long ulStartBlockIndex,
                 unsigned long ulRdBlockNumber)
{
    unsigned char ucRet;
    unsigned long i;
    
    xASSERT(pucDestBuf);
    
    //
    // Over the card range
    //
    if(ulStartBlockIndex + ulRdBlockNumber > g_sSDCardInfo.ulBlockNumber)
    {
        return SD_ERR_OVER_CARD_RANGE;        
    }
    
    //
    // Set Read Address
    //
    if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)
    {
        ucRet = SDMultipleBlockRead(ulStartBlockIndex);
    }
    else
    {
        ucRet = SDMultipleBlockRead(ulStartBlockIndex * SD_BLOCK_SIZE);   
    }
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
    
    for(i = 0; i < ulRdBlockNumber; i++)
    {
        ucRet = SDBlockDataRead(&pucDestBuf[i * SD_BLOCK_SIZE], SD_BLOCK_SIZE);    
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;   
        }
    }
    
    ucRet = SDTransmissionStop();
    
    //
    // Delay while busy
    //
    SDWaitBusy(SD_WAIT_READ);
    
    return ucRet;
    
    
}
#endif

//*****************************************************************************
//
//! \brief Write a block to the sdcard.
//!
//! \param pucSrcBuf is the source buffer to write.
//! \param ulBlockIndex is the block index to write. The index start from 0.
//!
//! This function is used to write a block data from the sdcard, a block length 
//! is always 512 bytes, defined by SD_BLOCK_SIZE.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDBlockWrite(const unsigned char *pucSrcBuf, unsigned long ulBlockIndex)
{
    unsigned char ucRet, pucTmp[2];
    
    //
    // Over the card range
    //
    if(ulBlockIndex >= g_sSDCardInfo.ulBlockNumber)
    {
        return SD_ERR_OVER_CARD_RANGE;   
    }  
    
    //
    // Set Write address
    //
    if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)
    {
        ucRet = SDSingleBlockWrite(ulBlockIndex);
    }
    else
    {
        ucRet = SDSingleBlockWrite(ulBlockIndex * SD_BLOCK_SIZE);   
    }
    
    //
    // Write Data
    //
    ucRet = SDBlockDataWrite((unsigned char*)pucSrcBuf, SD_BLOCK_SIZE, xfalse);
    
    if(ucRet == SD_NO_ERR)
    {
        ucRet = SDCardStatusRead(2, pucTmp);
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;   
        }
        
        if((pucTmp[0] != 0) || (pucTmp[1] != 0))
        {
            return SD_ERR_WRITE_BLK;   
        }
    }
    
    return ucRet;
}

#if SD_WRITE_MULTI_BLOCK_EN
//*****************************************************************************
//
//! \brief Write multi blocks to the sdcard.
//!
//! \param pucSrcBuf is the source buffer to write.
//! \param ulStartBlockIndex is the start block index to write. The index start
//! from 0.
//! \param ulWrBlockNumber is number of blocks to write.
//!
//! This function is used to write some blocks data to the sdcard, a block length 
//! is always 512 bytes, defined by SD_BLOCK_SIZE.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDMultiBlockWrite(const unsigned char* pucSrcBuf, 
                  unsigned long ulStartBlockIndex,
                  unsigned long ulWrBlockNumber)
{
    unsigned char ucRet;
    unsigned long i;
    
    xASSERT(pucSrcBuf);
    
    //
    // Over the card range
    //    
    if(ulStartBlockIndex + ulWrBlockNumber > g_sSDCardInfo.ulBlockNumber)
    {
        return SD_ERR_OVER_CARD_RANGE;
    }
    
    //
    // Set Write address
    //
    if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)
    {
        ucRet = SDMultipleBlockWrite(ulStartBlockIndex);
    }
    else
    {
        ucRet = SDMultipleBlockWrite(ulStartBlockIndex * SD_BLOCK_SIZE);
    }
    
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
    
    for(i = 0; i < ulWrBlockNumber; i++)
    {
        ucRet = SDBlockDataWrite((unsigned char*)&pucSrcBuf[i * SD_BLOCK_SIZE], 
                                 SD_BLOCK_SIZE, 
                                 xtrue);
        if(ucRet != SD_NO_ERR)
        {
            SDTransmissionStop();
            SDWaitBusy(SD_WAIT_WRITE);
            return ucRet;              
        }        
    }
    
    SDStopMultiToken();
    ucRet = SDWaitBusy(SD_WAIT_WRITE);
    if(ucRet != SD_NO_ERR)
    {
        return SD_ERR_TIMEOUT_WRITE;   
    }
    
    //
    // Get the block number that write correctly
    //
    ucRet = SDWrBlockNumberGet(&i);
    if(ucRet != SD_NO_ERR)
    {
        return ucRet;   
    }
    
    if(ulWrBlockNumber != i)
    {
        return SD_ERR_WRITE_BLK_NUMS;   
    }
    
    return SD_NO_ERR;
    
}
#endif

#if SD_ERASE_BLOCK_EN
//*****************************************************************************
//
//! \brief Erase some blocks.
//!
//! \param pucSrcBuf is the source buffer to write.
//! \param ulStartBlockIndex is the start block index to erase. The index start
//! from 0.
//! \param ulBlockNumber is number of blocks to erase.
//!
//! This function is used to write some blocks data to the sdcard, a block length 
//! is always 512 bytes, defined by SD_BLOCK_SIZE. \n
//! The erase unit defined by \ref tSDCardDeviceInfo.ulEraseUnit, is the block 
//! number that sdcard can erase max once.
//!
//! \return Returns SD_NO_ERR indicates everything is OK, others is the error 
//! code.
//
//*****************************************************************************
unsigned char
SDBlockErase(unsigned long ulStartBlockIndex, unsigned long ulBlockNumber)
{
    unsigned char ucRet;
    unsigned long ulRemain = ulBlockNumber;
    unsigned long ulEraseBlock = ulStartBlockIndex, ulEraseOnce;
    //
    // Over the card range
    //     
    if(ulStartBlockIndex + ulBlockNumber > g_sSDCardInfo.ulBlockNumber)   
    {
        return SD_ERR_OVER_CARD_RANGE;   
    }
    
    while(ulRemain)
    {
        ulEraseOnce = (ulRemain < g_sSDCardInfo.ulEraseUnit) ? ulRemain :
                                         g_sSDCardInfo.ulEraseUnit;   
        //
        // Set erase start address
        //
        if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)        
        {
            ucRet = SDEraseStartBlockSet(ulEraseBlock);
        }
        else
        {
            ucRet = SDEraseStartBlockSet(ulEraseBlock * SD_BLOCK_SIZE);
        }
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;
        }
        
        //
        // Set erase end address
        //
        if(g_sSDCardInfo.ulCardType == SD_TYPE_SDHC_SDXC)
        {
            ucRet = SDEraseEndBlockSet(ulEraseBlock + ulEraseOnce - 1);    
        }
        else
        {
            ucRet = SDEraseEndBlockSet((ulEraseBlock + ulEraseOnce - 1) * 
                                       SD_BLOCK_SIZE);    
        }
        
        //
        // execute erase
        //
        ucRet = SDEraseSelected();
        if(ucRet != SD_NO_ERR)
        {
            return ucRet;   
        }
        
        ulEraseBlock += ulEraseOnce;
        ulRemain -= ulEraseOnce;
    }
    
    return SD_NO_ERR;
}
#endif

//*****************************************************************************
//
//! \brief Get the sdcard info, such as total block number, CID and so on.
//!
//! \param None.
//!
//! This function should by called only after SDInit().
//!
//! \return Returns the sdcard info struct point.
//
//*****************************************************************************
const tSDCardDeviceInfo* 
SDCardInfoGet(void)
{
    return &g_sSDCardInfo;   
}
