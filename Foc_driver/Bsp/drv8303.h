#ifndef _drv8303_h_
#define _drv8303_h_
#include "main.h"
#include "stdbool.h"
#include "utils.h"
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include "stdio.h"
// Port
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;

typedef  uint16_t    DRV8301_Word_t;



#define DRV8301_DATA_MASK               (0x07FF)


#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)


//! \brief Enumeration for the register names
//!
typedef enum 
{
  DRV8301_RegName_Status_1  = 0 << 11,   //!< Status Register 1
  DRV8301_RegName_Status_2  = 1 << 11,   //!< Status Register 2
  DRV8301_RegName_Control_1 = 2 << 11,  //!< Control Register 1
  DRV8301_RegName_Control_2 = 3 << 11   //!< Control Register 2
} DRV8301_RegName_e;

// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum 
{
  DRV8301_CtrlMode_Read = 1 << 15,   //!< Read Mode
  DRV8301_CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8301_CtrlMode_e;



static inline DRV8301_Word_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                                   const DRV8301_RegName_e regName,
                                                   const uint16_t data)
{
  DRV8301_Word_t ctrlWord = ctrlMode | regName | (data & DRV8301_DATA_MASK);

  return(ctrlWord);
} 
//! \brief Defines the DRV8301 object
//!
typedef struct _DRV8301_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      EngpioHandle;               //!< the gpio handle that is connected to the drv8301 enable pin
  GPIO_Number_e    EngpioNumber;               //!< the gpio number that is connected to the drv8301 enable pin
  GPIO_Handle      nCSgpioHandle;              //!< the gpio handle that is connected to the drv8301 nCS pin
  GPIO_Number_e    nCSgpioNumber;               //!< the gpio number that is connected to the drv8301 nCS pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo
  bool             enableTimeOut;              //!< the timeout flag for drv8301 enable
} DRV8301_Obj;



//! \brief Enumeration for the Over Current modes
//!
typedef enum 
{
  DRV8301_OcMode_CurrentLimit  = 0 << 4,   //!< current limit when OC detected
  DRV8301_OcMode_LatchShutDown = 1 << 4,   //!< latch shut down when OC detected
  DRV8301_OcMode_ReportOnly    = 2 << 4,   //!< report only when OC detected
  DRV8301_OcMode_Disabled      = 3 << 4    //!< OC protection disabled
} DRV8301_OcMode_e;




//! \brief 保存读取到的寄存器的值
typedef struct _DRV_SPI_8301_Vars_t_
{
  uint16_t                  Stat_Reg_1_Value;
  uint16_t                  Stat_Reg_2_Value;
  uint16_t                  Ctrl_Reg_1_Value;
  uint16_t                  Ctrl_Reg_2_Value;
  bool                  SndCmd;
  bool                  RcvCmd;

}DRV_SPI_8301_Vars_t;
//! \brief Defines the DRV8301 handle
//!
typedef struct _DRV8301_Obj_ *DRV8301_Handle;
void Drv8303_Init();


#endif