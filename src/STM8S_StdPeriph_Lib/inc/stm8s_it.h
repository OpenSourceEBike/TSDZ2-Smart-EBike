/**
  ******************************************************************************
  * @file    stm8s_it.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    30-September-2014
  * @brief   This file contains the headers of the interrupt handlers
   ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_IT_H
#define __STM8S_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef _COSMIC_
 void _stext(void); /* RESET startup routine */
 INTERRUPT void NonHandledInterrupt(void);
#endif /* _COSMIC_ */

#if defined(_IAR_)
 INTERRUPT void TRAP_IRQHandler(void); /* TRAP */
 INTERRUPT void TLI_IRQHandler(void); /* TLI */
 INTERRUPT void AWU_IRQHandler(void); /* AWU */
 INTERRUPT void CLK_IRQHandler(void); /* CLOCK */
 INTERRUPT void EXTI_PORTA_IRQHandler(void); /* EXTI PORTA */
 INTERRUPT void EXTI_PORTB_IRQHandler(void); /* EXTI PORTB */
 INTERRUPT void EXTI_PORTC_IRQHandler(void); /* EXTI PORTC */
 INTERRUPT void EXTI_PORTD_IRQHandler(void); /* EXTI PORTD */
 INTERRUPT void EXTI_PORTE_IRQHandler(void); /* EXTI PORTE */
#elif defined(_SDCC_)
 INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler);
 INTERRUPT_HANDLER(TLI_IRQHandler, 0);
 INTERRUPT_HANDLER(AWU_IRQHandler, 1);
 INTERRUPT_HANDLER(CLK_IRQHandler, 2);
 INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3);
 INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4);
 INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5);
 INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6);
 INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7);
#endif

#if defined(STM8S903) || defined(STM8AF622x)
 #if defined(_IAR_)
 INTERRUPT void EXTI_PORTF_IRQHandler(void); /* EXTI PORTF */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8);
 #endif
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined (STM8AF52Ax)
 #if defined(_IAR_)
 INTERRUPT void CAN_RX_IRQHandler(void); /* CAN RX */
 INTERRUPT void CAN_TX_IRQHandler(void); /* CAN TX/ER/SC */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8);
 INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9);
 #endif
#endif /* (STM8S208) || (STM8AF52Ax) */

#if defined(_IAR_)
 INTERRUPT void SPI_IRQHandler(void); /* SPI */
 INTERRUPT void TIM1_CAP_COM_IRQHandler(void); /* TIM1 CAP/COM */
 INTERRUPT void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void); /* TIM1 UPD/OVF/TRG/BRK */
#elif defined(_SDCC_)
 INTERRUPT_HANDLER(SPI_IRQHandler, 10);
 INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11);
 INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12);
#endif

#if defined(STM8S903) || defined(STM8AF622x)
 #if defined(_IAR_)
 INTERRUPT void TIM5_UPD_OVF_BRK_TRG_IRQHandler(void); /* TIM5 UPD/OVF/BRK/TRG */
 INTERRUPT void TIM5_CAP_COM_IRQHandler(void); /* TIM5 CAP/COM */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13);
 INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14);
 #endif
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8A626x) */
 #if defined(_IAR_)
 INTERRUPT void TIM2_UPD_OVF_BRK_IRQHandler(void); /* TIM2 UPD/OVF/BRK */
 INTERRUPT void TIM2_CAP_COM_IRQHandler(void); /* TIM2 CAP/COM */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13);
 INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14);
 #endif
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 #if defined(_IAR_)
 INTERRUPT void TIM3_UPD_OVF_BRK_IRQHandler(void); /* TIM3 UPD/OVF/BRK */
 INTERRUPT void TIM3_CAP_COM_IRQHandler(void); /* TIM3 CAP/COM */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15);
 INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16);
 #endif
#endif /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8A626x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8S903)
 #if defined(_IAR_)
 INTERRUPT void UART1_TX_IRQHandler(void); /* UART1 TX */
 INTERRUPT void UART1_RX_IRQHandler(void); /* UART1 RX */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17);
 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18);
 #endif
#endif /* (STM8S208) || (STM8S207) || (STM8S903) || (STM8S103) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8AF622x)
 #if defined(_IAR_)
 INTERRUPT void UART4_TX_IRQHandler(void); /* UART4 TX */
 INTERRUPT void UART4_RX_IRQHandler(void); /* UART4 RX */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(UART4_TX_IRQHandler, 17);
 INTERRUPT_HANDLER(UART4_RX_IRQHandler, 18);
 #endif
#endif /* (STM8AF622x) */

 #if defined(_IAR_)
 INTERRUPT void I2C_IRQHandler(void); /* I2C */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(I2C_IRQHandler, 19);
 #endif

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
 #if defined(_IAR_)
 INTERRUPT void UART2_RX_IRQHandler(void); /* UART2 RX */
 INTERRUPT void UART2_TX_IRQHandler(void); /* UART2 TX */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20);
 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21);
 #endif
#endif /* (STM8S105) || (STM8AF626x) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 #if defined(_IAR_)
 INTERRUPT void UART3_RX_IRQHandler(void); /* UART3 RX */
 INTERRUPT void UART3_TX_IRQHandler(void); /* UART3 TX */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20);
 INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21);
 #endif
#endif /* (STM8S207) || (STM8S208) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 #if defined(_IAR_)
 INTERRUPT void ADC2_IRQHandler(void); /* ADC2 */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(ADC2_IRQHandler, 22);
#endif
#else /* (STM8S105) || (STM8S103) || (STM8S903) || (STM8AF622x) */
 #if defined(_IAR_)
 INTERRUPT void ADC1_IRQHandler(void); /* ADC1 */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(ADC1_IRQHandler, 22);
 #endif
#endif /* (STM8S207) || (STM8S208) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8S903) || defined(STM8AF622x)
 #if defined(_IAR_)
 INTERRUPT void TIM6_UPD_OVF_TRG_IRQHandler(void); /* TIM6 UPD/OVF/TRG */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23);
 #endif
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */
 #if defined(_IAR_)
 INTERRUPT void TIM4_UPD_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23);
 #endif
#endif /* (STM8S903) || (STM8AF622x) */
 #if defined(_IAR_)
 INTERRUPT void EEPROM_EEC_IRQHandler(void); /* EEPROM ECC CORRECTION */
 #elif defined(_SDCC_)
 INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24);
 #endif

#endif /* __STM8S_IT_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
