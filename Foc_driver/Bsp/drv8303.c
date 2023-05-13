#include "drv8303.h"

 DRV8301_Obj Driver_handle={
	          .spiHandle=&hspi3,
            .EngpioHandle =EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle =M0_CS_GPIO_Port,
            .nCSgpioNumber =M0_CS_Pin,
};

DRV_SPI_8301_Vars_t Drv8303_Reg={0};
// **************************************************************************
// the function prototypes


//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word


uint16_t DRV8301_readSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName)
{

uint16_t mask=0;
// Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking read
  uint16_t zerobuff = 0;
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);

  // Datasheet says you don't have to pulse the nCS between transfers, (16 clocks should commit the transfer)
  // but for some reason you actually need to pulse it.
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&zerobuff), (uint8_t*)(&recbuff), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);


  return (recbuff & DRV8301_DATA_MASK);
} 


void DRV8301_enable(DRV8301_Handle handle)
{

  //Enable driver
  HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

  //Wait for driver to come online
  delay_us(10);

  // Make sure the Fault bit is not set during startup
  while((DRV8301_readSpi(handle,DRV8301_RegName_Status_1) & DRV8301_STATUS1_FAULT_BITS) != 0);

  // Wait for the DRV8301 registers to update
  delay_us(1);

  return;
}


void DRV8301_writeSpi(DRV8301_Handle handle, const DRV8301_RegName_e regName,const uint16_t data)
{
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking write
  uint16_t controlword = (uint16_t)DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);

  return;
}

void DRV8301_writeData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;


  if(Spi_8301_Vars->SndCmd)
  {
    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = Spi_8301_Vars->Ctrl_Reg_1_Value;
                 
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);

		delay_us(10);
    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = Spi_8301_Vars->Ctrl_Reg_2_Value;
    DRV8301_writeSpi(handle,drvRegName,drvDataNew);
		delay_us(10);
    Spi_8301_Vars->SndCmd = false;
  }

  return;
} 


void DRV8301_setupSpi(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;


  Spi_8301_Vars->SndCmd = false;
  Spi_8301_Vars->RcvCmd = false;


  // Wait for the DRV8301 registers to update
  delay_us(10);

  // Update Status Register 1
  drvRegName = DRV8301_RegName_Status_1;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
	Spi_8301_Vars->Stat_Reg_1_Value=drvDataNew;
  delay_us(10);
	
  // Update Status Register 2
  drvRegName = DRV8301_RegName_Status_2;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
	Spi_8301_Vars->Stat_Reg_2_Value=drvDataNew;
	  delay_us(10);
  // Update Control Register 1
  drvRegName = DRV8301_RegName_Control_1;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
	Spi_8301_Vars->Ctrl_Reg_1_Value=drvDataNew;
  delay_us(10);
  // Update Control Register 2
  drvRegName = DRV8301_RegName_Control_2;
  drvDataNew = DRV8301_readSpi(handle,drvRegName);
	Spi_8301_Vars->Ctrl_Reg_2_Value=drvDataNew;
    delay_us(10);

  return;
}

void DRV8301_readData(DRV8301_Handle handle, DRV_SPI_8301_Vars_t *Spi_8301_Vars)
{
  DRV8301_RegName_e  drvRegName;
  uint16_t drvDataNew;

  if(Spi_8301_Vars->RcvCmd)
  {
    // Update Status Register 1
    drvRegName = DRV8301_RegName_Status_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_1_Value = drvDataNew;

    // Update Status Register 2
    drvRegName = DRV8301_RegName_Status_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Stat_Reg_2_Value = drvDataNew;

    // Update Control Register 1
    drvRegName = DRV8301_RegName_Control_1;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_1_Value = drvDataNew;

    // Update Control Register 2
    drvRegName = DRV8301_RegName_Control_2;
    drvDataNew = DRV8301_readSpi(handle,drvRegName);
    Spi_8301_Vars->Ctrl_Reg_2_Value = drvDataNew;
    Spi_8301_Vars->RcvCmd = false;
  }

  return;
} 


void Drv8303_Init()
{
	DRV8301_enable(&Driver_handle);
	DRV8301_setupSpi(&Driver_handle, &Drv8303_Reg);
	
  Drv8303_Reg.Ctrl_Reg_1_Value |= DRV8301_OcMode_LatchShutDown|21<<6;

  Drv8303_Reg.Ctrl_Reg_2_Value |= 1<<2;
	
	    Drv8303_Reg.SndCmd = true;
    DRV8301_writeData(&Driver_handle, &Drv8303_Reg);
    Drv8303_Reg.RcvCmd = true;
    DRV8301_readData(&Driver_handle, &Drv8303_Reg);
//		printf("sr1=%d\r\n",Drv8303_Reg.Stat_Reg_1_Value);
//	printf("sr2=%d\r\n",Drv8303_Reg.Stat_Reg_2_Value);
//	printf("c1=%d\r\n",Drv8303_Reg.Ctrl_Reg_1_Value);
//	printf("cr2=%d\r\n",Drv8303_Reg.Ctrl_Reg_2_Value);
	
}