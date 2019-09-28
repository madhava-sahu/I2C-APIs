#include "i2cdriver.h"

uint16_t AHBPrescale[8] = { 2, 4, 8, 16, 64, 128, 256, 512 }; //prescalers for AHB,Details from RCC registers
uint16_t APBPrescale[4] = { 2, 4, 8, 6 }; //prescalers for APB

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer,
		uint8_t Len, uint8_t SlaveAddr,uint8_t sr) {
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)))
		; //wait until the SB flag is set
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)))
		; //wait until the ADDR flag is set
	if (Len == 1) {
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			I2C_clearAddrFlag(pI2CHandle->pI2Cx);
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))); //wait until the RXNE flag is set
		if (sr==I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		(*pRxBuffer) = pI2CHandle->pI2Cx->DR;
	}

	if(Len >1){
		I2C_clearAddrFlag(pI2CHandle->pI2Cx);
		for (uint32_t i=Len;i>0;i--){
			while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))); //wait until the RXNE flag is set
			if(i==2){
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			if (sr==I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			(*pRxBuffer) = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}
void I2C_ManageAcking(I2C_TypeDef *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == I2C_ACK_ENABLE) {
		pI2Cx->CR1 |= I2C_CR1_ACK;
	} else {
		pI2Cx->CR1 &= (~I2C_CR1_ACK);
	}
}

void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx) {
	pI2Cx->CR1 |= (I2C_CR1_START);
}
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t flagName) {
	if (pI2Cx->SR1 & flagName) {
		return FLAG_SET;
	} else {
		return FLAG_RESET;
	}
}
void I2C_ExecuteAddressPhaseRead(I2C_TypeDef *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = (slaveAddr << 1); //Shift slave address 1 bit to accomodate the R/W bit
	slaveAddr |= 0x01; //make 0th bit = 1 so as to indicate Read mode
	pI2Cx->DR = slaveAddr;
}
void I2C_ExecuteAddressPhaseWrite(I2C_TypeDef *pI2Cx, uint8_t slaveAddr) {
	slaveAddr = (slaveAddr << 1); //Shift slave address 1 bit to accomodate the R/W bit
	slaveAddr &= (~1); //make 0th bit = 0 so as to indicate Write mode
	pI2Cx->DR = slaveAddr;
}
void I2C_clearAddrFlag(I2C_TypeDef *pI2Cx) {
	uint32_t dummyRead = 0;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
}
void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx) {
	pI2Cx->CR1 = (I2C_CR1_STOP);
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer,uint32_t Len, uint8_t slaveAddr,uint8_t sr) {
	
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)))
		; //wait until the SB flag is set
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))); //wait until the ADDR flag is set
	I2C_clearAddrFlag(pI2CHandle->pI2Cx);
	while (Len > 0) {
		while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))); //wait until the TXE flag is set
		*pTxbuffer = *pTxbuffer | 0x80;
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));
	if (sr==I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx); //Generate stop after confirming TXE,BTF is set
}

uint32_t RCC_getPCLKValue(void) {
	uint32_t pclk;
	uint8_t clksrc, temp, AHBPre, APBPre;
	uint32_t sysclk;
	clksrc = ((RCC->CFGR >> 2) & 0x03); //Get the clock source from RCC registers->right shift->mask out to get the value
	if (clksrc == 0) {
		sysclk = 16000000;   //internal Oscillator is used 16MHzs
	} else if (clksrc == 1) {
		sysclk = 8000000;   // External oscillator (HSE) is used which is 8MHz
	} else {
		//sysclk = GetPLLOut();   //PLL is used. Here we are not used PLL source.
	}
	//For getting the AHB prescaler from Registers,refer the RCC registers in manual
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		AHBPre = 1;  //sysclock is not divided
	} else {
		AHBPre = AHBPrescale[temp - 8];
	}
	//For getting the APB1 prescaler from RCC registers
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4) {
		APBPre = 1;   //AHB clock is not divided
	} else {
		APBPre = APBPrescale[temp - 4];
	}
	pclk = ((sysclk / AHBPre) / APBPre); //I2C peripheral clock is ((sysclk/AHBPre)/APBPre).
	return pclk;
}
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			//I2C1_PCLK_EN()
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
			;
		} else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN()
			;
		} else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN()
			;
		}
	} else {
		//TODO
	}

}
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pI2Cx->CR1 |= (I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(1 << 0);
	}

}
void I2C_Init(I2C_Handle_t *pI2CHandle) {


	/*CR1 configs*/
	//configure the ACK bit
	uint32_t tempreg = 0;
	uint32_t ccrval = 0;
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE); //Enable the I2C Peripheral clock
	I2C_PeripheralControl(I2C1,ENABLE);  //Enable the I2C peripheral
	tempreg = pI2CHandle->I2C_Config.I2C_AckControl <<10;  //Enable ACK
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	/*CR2 configs*/
	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg = ((RCC_getPCLKValue()/1000000) & 0x3F); //get pclk and mask
	pI2CHandle->pI2Cx->CR2 |= tempreg;

	/*OAR1 configs*/
	//program the device own address
	tempreg = 0;
	tempreg = (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1); //0 bit should not be touched. refer Manual
	pI2CHandle->pI2Cx->OAR1 |= tempreg;
	//put 14 bit as 1 as said by Manual
	tempreg = 0;
	tempreg = (1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	/*CCR register configs*/
	//configure the CCR value in CCR register using the formula in Manual
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) { //if SCL speed is lower or equal to 100kbps i.e standard mode
		ccrval = (RCC_getPCLKValue()/ (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed)); //to make it 12 bits out of 32bits
		tempreg = (ccrval & 0xFFF);
	} else {
		//if fast mode
		tempreg = (1 << 15); //configure the fast mode bit
		tempreg = 0;
		tempreg = (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14); //configure user provide DUTY bit
		tempreg = 0;
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) { //configure the CCR field value according to the formulae(refer Man)
			ccrval = (RCC_getPCLKValue()/ (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else {
			ccrval = (RCC_getPCLKValue()/ (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg = (ccrval & 0xFFF); //CCR value is 12bit so mask according to it
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;
	/* Program the TRISE register for rising time */
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		tempreg = (RCC_getPCLKValue() / 1000000) + 1;
	} else {
		tempreg = ((RCC_getPCLKValue() * 300) / 1000000000) + 1;
	}
	pI2CHandle->pI2Cx->TRISE |= (tempreg & 0x03F);

}
