/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
                        
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                    /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC3 GPIO Configuration    
    PF10     ------> ADC3_IN8
    PF9     ------> ADC3_IN7 
    */
    GPIO_InitStruct.Pin = Audio_IN_Pin|Potentiometer_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PF10     ------> ADC3_IN8
    PF9     ------> ADC3_IN7 
    */
    HAL_GPIO_DeInit(GPIOF, Audio_IN_Pin|Potentiometer_Pin);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }

}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC GPIO Configuration    
    PA4     ------> DAC_OUT1 
    */
    GPIO_InitStruct.Pin = Audio_DAC_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Audio_DAC_OUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }

}

/**
* @brief DAC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspDeInit 0 */

  /* USER CODE END DAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC_CLK_DISABLE();
  
    /**DAC GPIO Configuration    
    PA4     ------> DAC_OUT1 
    */
    HAL_GPIO_DeInit(Audio_DAC_OUT_GPIO_Port, Audio_DAC_OUT_Pin);

  /* USER CODE BEGIN DAC_MspDeInit 1 */

  /* USER CODE END DAC_MspDeInit 1 */
  }

}

/**
* @brief DCMI MSP Initialization
* This function configures the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DCMI_CLK_ENABLE();
  
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DCMI GPIO Configuration    
    PI7     ------> DCMI_D7
    PI6     ------> DCMI_D6
    PI5     ------> DCMI_VSYNC
    PI4     ------> DCMI_D5
    PH14     ------> DCMI_D4
    PH12     ------> DCMI_D3
    PH11     ------> DCMI_D2
    PH10     ------> DCMI_D1
    PH8     ------> DCMI_HSYNC
    PH9     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK 
    */
    GPIO_InitStruct.Pin = DCMI_D7_Pin|DCMI_D6_Pin|DCMI_VSYNC_Pin|DCMI_D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D2_Pin|DCMI_D1_Pin 
                          |DCMI_HSYNC_Pin|DCMI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = DCMI_PIXCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(DCMI_PIXCK_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN DCMI_MspInit 1 */

  /* USER CODE END DCMI_MspInit 1 */
  }

}

/**
* @brief DCMI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdcmi: DCMI handle pointer
* @retval None
*/
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi)
{
  if(hdcmi->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_CLK_DISABLE();
  
    /**DCMI GPIO Configuration    
    PI7     ------> DCMI_D7
    PI6     ------> DCMI_D6
    PI5     ------> DCMI_VSYNC
    PI4     ------> DCMI_D5
    PH14     ------> DCMI_D4
    PH12     ------> DCMI_D3
    PH11     ------> DCMI_D2
    PH10     ------> DCMI_D1
    PH8     ------> DCMI_HSYNC
    PH9     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK 
    */
    HAL_GPIO_DeInit(GPIOI, DCMI_D7_Pin|DCMI_D6_Pin|DCMI_VSYNC_Pin|DCMI_D5_Pin);

    HAL_GPIO_DeInit(GPIOH, DCMI_D4_Pin|DCMI_D3_Pin|DCMI_D2_Pin|DCMI_D1_Pin 
                          |DCMI_HSYNC_Pin|DCMI_D0_Pin);

    HAL_GPIO_DeInit(DCMI_PIXCK_GPIO_Port, DCMI_PIXCK_Pin);

  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }

}

/**
* @brief ETH MSP Initialization
* This function configures the hardware resources used in this example
* @param heth: ETH handle pointer
* @retval None
*/
void HAL_ETH_MspInit(ETH_HandleTypeDef* heth)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(heth->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ETH_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ETH GPIO Configuration    
    PB8     ------> ETH_TXD3
    PG14     ------> ETH_TXD1
    PG13     ------> ETH_TXD0
    PG11     ------> ETH_TX_EN
    PI10     ------> ETH_RX_ER
    PH2     ------> ETH_CRS
    PH3     ------> ETH_COL
    PC1     ------> ETH_MDC
    PC2     ------> ETH_TXD2
    PC3     ------> ETH_TX_CLK
    PH6     ------> ETH_RXD2
    PA1     ------> ETH_RX_CLK
    PC4     ------> ETH_RXD0
    PH7     ------> ETH_RXD3
    PA2     ------> ETH_MDIO
    PC5     ------> ETH_RXD1
    PA7     ------> ETH_RX_DV 
    */
    GPIO_InitStruct.Pin = MII_TXD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(MII_TXD3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MII_TXD1_Pin|MII_TXD0_Pin|MII_TX_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MII_RX_ER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(MII_RX_ER_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MII_CRS_Pin|MII_COL_Pin|MII_RXD2_Pin|MII_RXD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MII_MDC_Pin|MII_TXD2_Pin|MII_TX_CLK_Pin|MII_RXD0_Pin 
                          |MII_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MII_RX_CLK_RMII_REF_CLK_Pin|MII_MDIO_Pin|MII_RX_DV_RMII_CRSDV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ETH interrupt Init */
    HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  /* USER CODE BEGIN ETH_MspInit 1 */

  /* USER CODE END ETH_MspInit 1 */
  }

}

/**
* @brief ETH MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param heth: ETH handle pointer
* @retval None
*/
void HAL_ETH_MspDeInit(ETH_HandleTypeDef* heth)
{
  if(heth->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();
  
    /**ETH GPIO Configuration    
    PB8     ------> ETH_TXD3
    PG14     ------> ETH_TXD1
    PG13     ------> ETH_TXD0
    PG11     ------> ETH_TX_EN
    PI10     ------> ETH_RX_ER
    PH2     ------> ETH_CRS
    PH3     ------> ETH_COL
    PC1     ------> ETH_MDC
    PC2     ------> ETH_TXD2
    PC3     ------> ETH_TX_CLK
    PH6     ------> ETH_RXD2
    PA1     ------> ETH_RX_CLK
    PC4     ------> ETH_RXD0
    PH7     ------> ETH_RXD3
    PA2     ------> ETH_MDIO
    PC5     ------> ETH_RXD1
    PA7     ------> ETH_RX_DV 
    */
    HAL_GPIO_DeInit(MII_TXD3_GPIO_Port, MII_TXD3_Pin);

    HAL_GPIO_DeInit(GPIOG, MII_TXD1_Pin|MII_TXD0_Pin|MII_TX_EN_Pin);

    HAL_GPIO_DeInit(MII_RX_ER_GPIO_Port, MII_RX_ER_Pin);

    HAL_GPIO_DeInit(GPIOH, MII_CRS_Pin|MII_COL_Pin|MII_RXD2_Pin|MII_RXD3_Pin);

    HAL_GPIO_DeInit(GPIOC, MII_MDC_Pin|MII_TXD2_Pin|MII_TX_CLK_Pin|MII_RXD0_Pin 
                          |MII_RXD1_Pin);

    HAL_GPIO_DeInit(GPIOA, MII_RX_CLK_RMII_REF_CLK_Pin|MII_MDIO_Pin|MII_RX_DV_RMII_CRSDV_Pin);

    /* ETH interrupt DeInit */
    HAL_NVIC_DisableIRQ(ETH_IRQn);
  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }

}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration    
    PB9     ------> I2C1_SDA
    PB6     ------> I2C1_SCL 
    */
    GPIO_InitStruct.Pin = I2C1_SDA_Pin|I2C1_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB9     ------> I2C1_SDA
    PB6     ------> I2C1_SCL 
    */
    HAL_GPIO_DeInit(GPIOB, I2C1_SDA_Pin|I2C1_SCL_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }

}

/**
* @brief SD MSP Initialization
* This function configures the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspInit(SD_HandleTypeDef* hsd)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hsd->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspInit 0 */

  /* USER CODE END SDIO_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SDIO_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SDIO GPIO Configuration    
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    PC9     ------> SDIO_D1
    PC8     ------> SDIO_D0 
    */
    GPIO_InitStruct.Pin = MicroSDCard_CLK_Pin|MicroSDCard_D1_Pin|MicroSDCard_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = MicroSDCard_CMD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    HAL_GPIO_Init(MicroSDCard_CMD_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SDIO_MspInit 1 */

  /* USER CODE END SDIO_MspInit 1 */
  }

}

/**
* @brief SD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hsd: SD handle pointer
* @retval None
*/
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd)
{
  if(hsd->Instance==SDIO)
  {
  /* USER CODE BEGIN SDIO_MspDeInit 0 */

  /* USER CODE END SDIO_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDIO_CLK_DISABLE();
  
    /**SDIO GPIO Configuration    
    PC12     ------> SDIO_CK
    PD2     ------> SDIO_CMD
    PC9     ------> SDIO_D1
    PC8     ------> SDIO_D0 
    */
    HAL_GPIO_DeInit(GPIOC, MicroSDCard_CLK_Pin|MicroSDCard_D1_Pin|MicroSDCard_D0_Pin);

    HAL_GPIO_DeInit(MicroSDCard_CMD_GPIO_Port, MicroSDCard_CMD_Pin);

  /* USER CODE BEGIN SDIO_MspDeInit 1 */

  /* USER CODE END SDIO_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM3 GPIO Configuration    
    PC6     ------> TIM3_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}
/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX 
    */
    GPIO_InitStruct.Pin = MicroSDCard_D3_Pin|MicroSDCard_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PC11     ------> USART3_RX
    PC10     ------> USART3_TX 
    */
    HAL_GPIO_DeInit(GPIOC, MicroSDCard_D3_Pin|MicroSDCard_D2_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }

}

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void){
  /* USER CODE BEGIN FSMC_MspInit 0 */

  /* USER CODE END FSMC_MspInit 0 */
  GPIO_InitTypeDef GPIO_InitStruct ={0};
  if (FSMC_Initialized) {
    return;
  }
  FSMC_Initialized = 1;

  /* Peripheral clock enable */
  __HAL_RCC_FSMC_CLK_ENABLE();
  
  /** FSMC GPIO Configuration  
  PE3   ------> FSMC_A19
  PE1   ------> FSMC_NBL1
  PE0   ------> FSMC_NBL0
  PD7   ------> FSMC_NE1
  PE4   ------> FSMC_A20
  PB7   ------> FSMC_NL
  PG10   ------> FSMC_NE3
  PD6   ------> FSMC_NWAIT
  PD0   ------> FSMC_D2
  PG9   ------> FSMC_NE2
  PD5   ------> FSMC_NWE
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD3   ------> FSMC_CLK
  PF0   ------> FSMC_A0
  PF2   ------> FSMC_A2
  PF1   ------> FSMC_A1
  PF3   ------> FSMC_A3
  PF4   ------> FSMC_A4
  PF5   ------> FSMC_A5
  PG5   ------> FSMC_A15
  PG4   ------> FSMC_A14
  PG3   ------> FSMC_A13
  PD15   ------> FSMC_D1
  PG2   ------> FSMC_A12
  PG1   ------> FSMC_A11
  PD14   ------> FSMC_D0
  PD13   ------> FSMC_A18
  PF13   ------> FSMC_A7
  PG0   ------> FSMC_A10
  PE13   ------> FSMC_D10
  PD12   ------> FSMC_A17
  PD11   ------> FSMC_A16
  PD10   ------> FSMC_D15
  PF12   ------> FSMC_A6
  PF15   ------> FSMC_A9
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE11   ------> FSMC_D8
  PE14   ------> FSMC_D11
  PD9   ------> FSMC_D14
  PD8   ------> FSMC_D13
  PF14   ------> FSMC_A8
  PE7   ------> FSMC_D4
  PE10   ------> FSMC_D7
  PE12   ------> FSMC_D9
  PE15   ------> FSMC_D12
  */
  GPIO_InitStruct.Pin = A19_Pin|FSMC_NBL1_Pin|FSMC_NBL0_Pin|A20_Pin 
                          |D10_Pin|D5_Pin|D6_Pin|D8_Pin 
                          |D11_Pin|D4_Pin|D7_Pin|D9_Pin 
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FSMC_NE1_Pin|FSMC_NWAIT_Pin|D2_Pin|FSMC_NWE_Pin 
                          |D3_Pin|FSMC_NOE_Pin|FSMC_CLK_Pin|D1_Pin 
                          |D0_Pin|A18_Pin|A17_Pin|A16_Pin 
                          |D15_Pin|D14_Pin|D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FSMC_NL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(FSMC_NL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FSMC_NE3_Pin|FSMC_NE2_Pin|A15_Pin|A14_Pin 
                          |A13_Pin|A12_Pin|A11_Pin|A10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = A0_Pin|A2_Pin|A1_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|A7_Pin|A6_Pin 
                          |A9_Pin|A8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN FSMC_MspInit 1 */

  /* USER CODE END FSMC_MspInit 1 */
}

void HAL_NOR_MspInit(NOR_HandleTypeDef* hnor){
  /* USER CODE BEGIN NOR_MspInit 0 */

  /* USER CODE END NOR_MspInit 0 */
  HAL_FSMC_MspInit();
  /* USER CODE BEGIN NOR_MspInit 1 */

  /* USER CODE END NOR_MspInit 1 */
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram){
  /* USER CODE BEGIN SRAM_MspInit 0 */

  /* USER CODE END SRAM_MspInit 0 */
  HAL_FSMC_MspInit();
  /* USER CODE BEGIN SRAM_MspInit 1 */

  /* USER CODE END SRAM_MspInit 1 */
}

static uint32_t FSMC_DeInitialized = 0;

static void HAL_FSMC_MspDeInit(void){
  /* USER CODE BEGIN FSMC_MspDeInit 0 */

  /* USER CODE END FSMC_MspDeInit 0 */
  if (FSMC_DeInitialized) {
    return;
  }
  FSMC_DeInitialized = 1;
  /* Peripheral clock enable */
  __HAL_RCC_FSMC_CLK_DISABLE();
  
  /** FSMC GPIO Configuration  
  PE3   ------> FSMC_A19
  PE1   ------> FSMC_NBL1
  PE0   ------> FSMC_NBL0
  PD7   ------> FSMC_NE1
  PE4   ------> FSMC_A20
  PB7   ------> FSMC_NL
  PG10   ------> FSMC_NE3
  PD6   ------> FSMC_NWAIT
  PD0   ------> FSMC_D2
  PG9   ------> FSMC_NE2
  PD5   ------> FSMC_NWE
  PD1   ------> FSMC_D3
  PD4   ------> FSMC_NOE
  PD3   ------> FSMC_CLK
  PF0   ------> FSMC_A0
  PF2   ------> FSMC_A2
  PF1   ------> FSMC_A1
  PF3   ------> FSMC_A3
  PF4   ------> FSMC_A4
  PF5   ------> FSMC_A5
  PG5   ------> FSMC_A15
  PG4   ------> FSMC_A14
  PG3   ------> FSMC_A13
  PD15   ------> FSMC_D1
  PG2   ------> FSMC_A12
  PG1   ------> FSMC_A11
  PD14   ------> FSMC_D0
  PD13   ------> FSMC_A18
  PF13   ------> FSMC_A7
  PG0   ------> FSMC_A10
  PE13   ------> FSMC_D10
  PD12   ------> FSMC_A17
  PD11   ------> FSMC_A16
  PD10   ------> FSMC_D15
  PF12   ------> FSMC_A6
  PF15   ------> FSMC_A9
  PE8   ------> FSMC_D5
  PE9   ------> FSMC_D6
  PE11   ------> FSMC_D8
  PE14   ------> FSMC_D11
  PD9   ------> FSMC_D14
  PD8   ------> FSMC_D13
  PF14   ------> FSMC_A8
  PE7   ------> FSMC_D4
  PE10   ------> FSMC_D7
  PE12   ------> FSMC_D9
  PE15   ------> FSMC_D12
  */
  HAL_GPIO_DeInit(GPIOE, A19_Pin|FSMC_NBL1_Pin|FSMC_NBL0_Pin|A20_Pin 
                          |D10_Pin|D5_Pin|D6_Pin|D8_Pin 
                          |D11_Pin|D4_Pin|D7_Pin|D9_Pin 
                          |D12_Pin);

  HAL_GPIO_DeInit(GPIOD, FSMC_NE1_Pin|FSMC_NWAIT_Pin|D2_Pin|FSMC_NWE_Pin 
                          |D3_Pin|FSMC_NOE_Pin|FSMC_CLK_Pin|D1_Pin 
                          |D0_Pin|A18_Pin|A17_Pin|A16_Pin 
                          |D15_Pin|D14_Pin|D13_Pin);

  HAL_GPIO_DeInit(FSMC_NL_GPIO_Port, FSMC_NL_Pin);

  HAL_GPIO_DeInit(GPIOG, FSMC_NE3_Pin|FSMC_NE2_Pin|A15_Pin|A14_Pin 
                          |A13_Pin|A12_Pin|A11_Pin|A10_Pin);

  HAL_GPIO_DeInit(GPIOF, A0_Pin|A2_Pin|A1_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|A7_Pin|A6_Pin 
                          |A9_Pin|A8_Pin);

  /* USER CODE BEGIN FSMC_MspDeInit 1 */

  /* USER CODE END FSMC_MspDeInit 1 */
}

void HAL_NOR_MspDeInit(NOR_HandleTypeDef* hnor){
  /* USER CODE BEGIN NOR_MspDeInit 0 */

  /* USER CODE END NOR_MspDeInit 0 */
  HAL_FSMC_MspDeInit();
  /* USER CODE BEGIN NOR_MspDeInit 1 */

  /* USER CODE END NOR_MspDeInit 1 */
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram){
  /* USER CODE BEGIN SRAM_MspDeInit 0 */

  /* USER CODE END SRAM_MspDeInit 0 */
  HAL_FSMC_MspDeInit();
  /* USER CODE BEGIN SRAM_MspDeInit 1 */

  /* USER CODE END SRAM_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
