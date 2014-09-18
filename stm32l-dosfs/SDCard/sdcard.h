//*****************************************************************************
//
//! \file sdcard.h
//! \brief Prototypes for SDCard Driver(Based on CoX Peripheral Interface).
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
#ifndef __SDCARD_H__
#define __SDCARD_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
    
//*****************************************************************************
//
//! \addtogroup CoX_Driver_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Memory
//! @{
//
//*****************************************************************************
    
//*****************************************************************************
//
//! \addtogroup Memory_SDCard
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SDCard_SDSC_SDHC_SDXC
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SDCard_Config SD Card SPI Driver Configurtion
//!
//! \brief Configurtions such as the GPIO Pin used should be set before using
//! this driver.
//!    
//! @{
//
//*****************************************************************************

//
//! if enable CRC
//
#define SD_CRC_EN               0

//
//! if enable multi block read function
//
#define SD_READ_MULTI_BLOCK_EN  1

//
//! if enable multi block write function
//
#define SD_WRITE_MULTI_BLOCK_EN 1

//
//! if enable block erase function
//
#define SD_ERASE_BLOCK_EN       1

//
//! SD Card Power Pin
//
#define SD_PIN_POWER            PD12

//
//! SD Card Power Pin Active State
//
#define SD_PIN_POWER_ACTIVE     0

//
//! SD Card Pin1 - CS(Chip Select (active low))
//
#define SD_PIN_CS               PC8

//
//! SD Card Pin2 - DataIn(Host-to-card Commands and Data)
//
#define SD_PIN_DATAIN           PC11

//
//! SD Card Pin5 - CLK(Clock)
//
#define SD_PIN_CLK              PC9

//
//! SD Card Pin7 - DataOut(Card-to-host Data and Status)
//
#define SD_PIN_DATAOUT          PC10

//
//! SD SPI max(normal) clock freq
//
#define SD_SPI_CLOCK            1000000

//
//! SD Card Host SPI Port
//
#define SD_HOST_SPI_PORT        xSPI1_BASE
#define SD_HOST_SPI_PIN_CLK     SPI1CLK
#define SD_HOST_SPI_PIN_MOSI    SPI1MOSI
#define SD_HOST_SPI_PIN_MISO    SPI1MISO

//*****************************************************************************
//
//! @}
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SDCard_Info_Struct SD Card Info Struct   
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SDCard_Type SD Card Type
//! @{
//
//*****************************************************************************

#define SD_TYPE_SDSC_V1         0
#define SD_TYPE_SDSC_V2         1
#define SD_TYPE_SDHC_SDXC       2

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//
//! SD Card Block Size, should be 512
//
#define SD_BLOCK_SIZE           512        

//
//! SD Card Block bit length, 9 means 512byte
//
#define SD_BLOCK_SIZE_NBITS     9 

typedef struct 
{
    //
    // Card Type
    //
    unsigned long ulCardType;
    
    //
    //! Total Block Number(Block size is 512)
    //
    unsigned long ulBlockNumber;
    
    //
    // Erase Unit(Block Number)
    //
    unsigned long ulEraseUnit;
    
    unsigned long ulReadTimeout;
    unsigned long ulWriteTimeout;
    unsigned long ulEraseTimeout;
    
    //
    //! CID register
    //
    unsigned char pucCID[16];
}tSDCardDeviceInfo;

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SDCard_Error_Code SD Card Error Code  
//! @{
//
//*****************************************************************************

//
// No error occurs
//
#define SD_NO_ERR               0x00
#define SD_ERR_NO_CARD          0x01
#define SD_ERR_USER_PARAM       0x02
#define SD_ERR_CARD_PARAM       0x03

//
//! voltage is not support 
//
#define SD_ERR_VOL_NOT_SUSP     0x04
#define SD_ERR_OVER_CARD_RANGE                                                \
                                0x05
//
// SD Card Command error code
//
#define SD_ERR_CMD_RESP_TYPE    0x10
#define SD_ERR_CMD_TIMEOUT      0x11
#define SD_ERR_CMD_RESP         0x12

//
// Data error code
//
#define SD_ERR_DATA_CRC16       0x20
#define SD_ERR_DATA_START_TOK   0x21
#define SD_ERR_DATA_RESP        0x22

//
// Wait error code
//
#define SD_ERR_TIMEOUT_WAIT     0x30
#define SD_ERR_TIMEOUT_READ     0x31
#define SD_ERR_TIMEOUT_WRITE    0x32
#define SD_ERR_TIMEOUT_ERASE    0x33
#define SD_ERR_TIMEOUT_WAIT_IDLE                                              \
                                0x34

//
// Write error code
//
#define SD_ERR_WRITE_BLK        0x40
#define SD_ERR_WRITE_BLK_NUMS   0x41
#define SD_ERR_WRITE_PROTECT    0x42

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SDCard_API SD Card API
//!    
//! @{
//
//*****************************************************************************
extern unsigned char SDInit(void);
extern unsigned char SDBlockRead(unsigned char *pucDestBuf, 
                                 unsigned long ulBlockIndex);
extern unsigned char SDBlockWrite(const unsigned char *pucSrcBuf, 
                                  unsigned long ulBlockIndex);
#if SD_READ_MULTI_BLOCK_EN
extern unsigned char SDMultiBlockRead(unsigned char *pucDestBuf, 
                                      unsigned long ulStartBlockIndex,
                                      unsigned long ulRdBlockNumber);
#endif

#if SD_WRITE_MULTI_BLOCK_EN
extern unsigned char SDMultiBlockWrite(const unsigned char* pucSrcBuf, 
                                       unsigned long ulStartBlockIndex,
                                       unsigned long ulWrBlockNumber);
#endif

#if SD_ERASE_BLOCK_EN
extern unsigned char SDBlockErase(unsigned long ulStartBlockIndex, 
                                  unsigned long ulBlockNumber);
#endif

extern const tSDCardDeviceInfo* SDCardInfoGet(void);

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
