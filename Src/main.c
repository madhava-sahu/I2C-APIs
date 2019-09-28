	#include "main.h"
	void HAL_I2C_Mem_Read( I2C_Handle_t *pI2CHandle,  uint8_t SlaveAddr,  uint8_t RegAddr,uint8_t *Rxbuffer, uint32_t Size);
	void I2C_RequestMemoryRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr);

	#define SLAVE_ADDR 0x5F
	I2C_Handle_t hi2c;
	uint8_t rxbuffer[4] ={0};
	uint8_t MemAddr = 0x3C;
	uint8_t respnose = 0;
	int main(){
		I2C_GpioInit();
		I2C_userInit();
		uint32_t Size = 4;
		int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
		int16_t T0_degC, T1_degC;
		uint8_t buffer[4], tmp;
		float   tmp_f;
		HAL_I2C_Mem_Read( &hi2c,  SLAVE_ADDR,  (uint8_t)0x32,buffer, 2);
		HAL_I2C_Mem_Read( &hi2c,  SLAVE_ADDR,  (uint8_t)0x35,&tmp,1);
		//readReg();
		while(1){

		}
	}
	void I2C_RequestMemoryRead(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t RegAddr){
	 /* Enable Acknowledge */
  pI2CHandle->pI2Cx->CR1 |= I2C_CR1_ACK;

  /* Generate Start */
  pI2CHandle->pI2Cx->CR1 |= I2C_CR1_START;

  /* Wait until SB flag is set */
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

  /* Send slave address */
 I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

  /* Wait until ADDR flag is set */
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

  /* Clear ADDR flag */
	I2C_clearAddrFlag(pI2CHandle->pI2Cx);

  /* Wait until TXE flag is set */
 while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
 pI2CHandle->pI2Cx->DR = RegAddr;
  
  /* Wait until TXE flag is set */
  while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));

  /* Generate Restart */
  pI2CHandle->pI2Cx->CR1 |= I2C_CR1_START;

  /* Wait until SB flag is set */
  	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)))

  /* Send slave address */
   I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

  /* Wait until ADDR flag is set */
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));		
	}
	void HAL_I2C_Mem_Read( I2C_Handle_t *pI2CHandle,  uint8_t SlaveAddr,  uint8_t RegAddr,uint8_t *Rxbuffer, uint32_t Size){
		if (Size >1){
			RegAddr |= 0x80;
		}
			/* Check if the I2C is already enabled */
		if((pI2CHandle->pI2Cx->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
		{
				/* Enable I2C peripheral */
			I2C_PeripheralControl(I2C1,ENABLE);  //Enable the I2C peripheral
		}
				/* Disable Pos */
			pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_POS;
		I2C_RequestMemoryRead(pI2CHandle, SlaveAddr, RegAddr);
		
		if(Size == 0U)
			{
				/* Clear ADDR flag */
				I2C_clearAddrFlag(pI2CHandle->pI2Cx);
				
				/* Generate Stop */
			 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			else if(Size == 1U)
			{
				/* Disable Acknowledge */
				pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_ACK;

				/* Clear ADDR flag */
			I2C_clearAddrFlag(pI2CHandle->pI2Cx);

				/* Generate Stop */
				pI2CHandle->pI2Cx->CR1 |= I2C_CR1_STOP;
			}
			else if(Size == 2U)
			{
				/* Disable Acknowledge */
				pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_ACK;

				/* Enable Pos */
				pI2CHandle->pI2Cx->CR1 |= I2C_CR1_POS;

				/* Clear ADDR flag */
			 I2C_clearAddrFlag(pI2CHandle->pI2Cx);
			}
			else
			{
				/* Clear ADDR flag */
				I2C_clearAddrFlag(pI2CHandle->pI2Cx);
			}

			while(Size > 0U)
			{
				if(Size <= 3U)
				{
					/* One byte */
					if(Size== 1U)
					{
						/* Wait until RXNE flag is set */
						while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
						/* Read data from DR */
					 (*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
					 Size--;
					}
					/* Two bytes */
					else if(Size == 2U)
					{
						/* Wait until BTF flag is set */
						while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

						/* Generate Stop */
						pI2CHandle->pI2Cx->CR1 |= I2C_CR1_STOP;

						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;

						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;
					}
					/* 3 Last bytes */
					else
					{
						/* Wait until BTF flag is set */
						while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

						/* Disable Acknowledge */
						pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_ACK;

						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;

						/* Wait until BTF flag is set */
						while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

						/* Generate Stop */
						pI2CHandle->pI2Cx->CR1 |= I2C_CR1_STOP;

						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;

						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;
					}
				}
				else
				{
					/* Wait until RXNE flag is set */
				while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

					/* Read data from DR */
					(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
					Size--;

					if((pI2CHandle->pI2Cx->SR1 & I2C_SR1_BTF) != I2C_SR1_BTF)
					{
						/* Read data from DR */
						(*Rxbuffer++) = pI2CHandle->pI2Cx->DR;
						Size--;
					}
				}
			}
		
	}
	void readReg(void){
		I2C_MasterSendData(&hi2c,&MemAddr,1, SLAVE_ADDR,I2C_ENABLE_SR);
		I2C_MasterReceiveData(&hi2c,rxbuffer,4,SLAVE_ADDR,I2C_DISABLE_SR);
	}
	void I2C_userInit(void){
		hi2c.pI2Cx = I2C1;
		hi2c.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
		hi2c.I2C_Config.I2C_DeviceAddress = 0x61;
		hi2c.I2C_Config.I2C_FMDutyCycle =I2C_FM_DUTY_2;
		hi2c.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
		I2C_Init(&hi2c);
	}
	void I2C_GpioInit(void){
		RCC->AHB1ENR |= (1<<1);
		GPIOB->MODER |= (1<<17);
		GPIOB->MODER |= (1<<19);
		GPIOB->OTYPER |= (1<<8);
		GPIOB->OTYPER |= (1<<9);
		GPIOB->PUPDR |= (1<<16);
		GPIOB->PUPDR |= (1<<18);
		GPIOB->AFR[1] |= (1<<2);
		GPIOB->AFR[1] |= (1<<6);
	}
