/********************************************************************************************************//**
* @file timer_driver.c
*
* @brief File containing the APIs for configuring the TIM peripheral.
*
* Public Functions:
*       - void     Timer_Init(Timer_Handle_t* Timer_Handle)
*       - void     Timer_Start(Timer_Handle_t* Timer_Handle)
*       - void     Timer_Stop(Timer_Handle_t* Timer_Handle)
*       - void     Timer_ICInit(Timer_Handle_t* Timer_Handle, IC_Handle_t IC_Handle, CC_Channel_t channel)
*       - uint32_t Timer_CCGetValue(Timer_Handle_t* Timer_Handle, CC_Channel_t channel)
*       - void     Timer_CCSetValue(Timer_Handle_t* Timer_Handle, CC_Channel_t channel, uint32_t value)
*       - void     Timer_PerClkCtrl(TIM_TypeDef* pTimer, uint8_t en_or_di)
*       - void     Timer_IRQConfig(uint8_t IRQNumber, uint8_t en_or_di)
*       - void     Timer_IRQHandling(Timer_Handle_t* Timer_Handle)
*       - void     Timer_ApplicationEventCallback(void)
*
* @note
*       For further information about functions refer to the corresponding header file.
*/

#include <stdint.h>
#include "hal_timer.h"
#include "stm32f429xx.h"
#include "stm32f4xx.h"

/***********************************************************************************************************/
/*                                       Public API Definitions                                            */
/***********************************************************************************************************/

void Timer_Init(Timer_Handle_t* Timer_Handle){

    /* Enable peripheral clock */
    Timer_PerClkCtrl(Timer_Handle->pTimer, ENABLE);
    /* Clear and set prescaler value */
    MODIFY_REG(Timer_Handle->pTimer->PSC, 0xFFFF, Timer_Handle->prescaler);
    /*Clear and set period value */
    /* TIM2 and TIM5 have 32 bits of counter capacity */
    if((Timer_Handle->pTimer == TIM2) || (Timer_Handle->pTimer == TIM5)){
        MODIFY_REG(Timer_Handle->pTimer->ARR, 0xFFFFFFFF, Timer_Handle->period);
    }
    /* Rest of the TIMx have 16 bits of counter capacity */
    else{
        MODIFY_REG(Timer_Handle->pTimer->ARR, 0xFFFF, Timer_Handle->period);
    }
    /* Enable interrupt */
    SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_UIE);
}

void Timer_Start(Timer_Handle_t* Timer_Handle){

    /* Clear UIF bit in SR register */
    CLEAR_BIT(Timer_Handle->pTimer->SR, TIM_SR_UIF);
    /* Set counter enable */
    SET_BIT(Timer_Handle->pTimer->CR1, TIM_CR1_CEN);
}

void Timer_Stop(Timer_Handle_t* Timer_Handle){

    /* Set counter disable */
    CLEAR_BIT(Timer_Handle->pTimer->CR1, TIM_CR1_CEN);
}

void Timer_ICInit(Timer_Handle_t* Timer_Handle, IC_Handle_t IC_Handle, CC_Channel_t channel){

    switch(channel){
        case CHANNEL1:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x05 << TIM_CCER_CC1P_Pos,
                       IC_Handle.ic_polarity << TIM_CCER_CC1P_Pos
                      );
            /* Set input capture selection */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x3 << TIM_CCMR1_CC1S_Pos,
                       IC_Handle.ic_select << TIM_CCMR1_CC1S_Pos
                      );
            /* Set input capture prescaler */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x3 << TIM_CCMR1_IC1PSC_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR1_IC1PSC_Pos
                      );
            /* Set input capture filter */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0xF << TIM_CCMR1_IC1F_Pos,
                       IC_Handle.ic_filter << TIM_CCMR1_IC1F_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC1IE);
            /* Enable capture/compare 1 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC1E);
            break;
        case CHANNEL2:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x05 << TIM_CCER_CC2P_Pos,
                       IC_Handle.ic_polarity << TIM_CCER_CC2P_Pos
                      );
            /* Set input capture selection */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x3 << TIM_CCMR1_CC2S_Pos,
                       IC_Handle.ic_select << TIM_CCMR1_CC2S_Pos
                      );
            /* Set input capture prescaler */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x3 << TIM_CCMR1_IC2PSC_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR1_IC2PSC_Pos
                      );
            /* Set input capture filter */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0xF << TIM_CCMR1_IC2F_Pos,
                       IC_Handle.ic_filter << TIM_CCMR1_IC2F_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC2IE);
            /* Enable capture/compare 2 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC2E);
            break;
        case CHANNEL3:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x05 << TIM_CCER_CC3P_Pos,
                       IC_Handle.ic_polarity << TIM_CCER_CC3P_Pos
                      );
            /* Set input capture selection */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x3 << TIM_CCMR2_CC3S_Pos,
                       IC_Handle.ic_select << TIM_CCMR2_CC3S_Pos
                      );
            /* Set input capture prescaler */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x3 << TIM_CCMR2_IC3PSC_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR2_IC3PSC_Pos
                      );
            /* Set input capture filter */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0xF << TIM_CCMR2_IC3F_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR2_IC3F_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC3IE);
            /* Enable capture/compare 3 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC3E);
            break;
        case CHANNEL4:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x05 << TIM_CCER_CC4P_Pos,
                       IC_Handle.ic_polarity << TIM_CCER_CC4P_Pos
                      );
            /* Set input capture selection */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x3 << TIM_CCMR2_CC4S_Pos,
                       IC_Handle.ic_select << TIM_CCMR2_CC4S_Pos
                      );
            /* Set input capture prescaler */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x3 << TIM_CCMR2_IC4PSC_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR2_IC4PSC_Pos
                      );
            /* Set input capture filter */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0xF << TIM_CCMR2_IC4F_Pos,
                       IC_Handle.ic_prescaler << TIM_CCMR2_IC4F_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC4IE);
            /* Enable capture/compare 3 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC4E);
            break;
        default:
            break;
    }
}

uint32_t Timer_CCGetValue(Timer_Handle_t* Timer_Handle, CC_Channel_t channel){

    uint32_t ret_val = 0;

    switch(channel){
        case CHANNEL1:
            ret_val = Timer_Handle->pTimer->CCR1;
            break;
        case CHANNEL2:
            ret_val = Timer_Handle->pTimer->CCR2;
            break;
        case CHANNEL3:
            ret_val = Timer_Handle->pTimer->CCR3;
            break;
        case CHANNEL4:
            ret_val = Timer_Handle->pTimer->CCR4;
            break;
        default:
            break;
    }

    return ret_val;
}

void Timer_CCSetValue(Timer_Handle_t* Timer_Handle, CC_Channel_t channel, uint32_t value){

    /* Disable timer */
    CLEAR_BIT(Timer_Handle->pTimer->CR1, TIM_CR1_CEN);

    switch(channel){
        case CHANNEL1:
            MODIFY_REG(Timer_Handle->pTimer->CCR1, 0xFFFFFFFF, value);
            break;
        case CHANNEL2:
            MODIFY_REG(Timer_Handle->pTimer->CCR2, 0xFFFFFFFF, value);
            break;
        case CHANNEL3:
            MODIFY_REG(Timer_Handle->pTimer->CCR3, 0xFFFFFFFF, value);
            break;
        case CHANNEL4:
            MODIFY_REG(Timer_Handle->pTimer->CCR4, 0xFFFFFFFF, value);
            break;
        default:
            break;
    }

    /* Enable timer */
    SET_BIT(Timer_Handle->pTimer->CR1, TIM_CR1_CEN);
}

void Timer_OCInit(Timer_Handle_t* Timer_Handle, OC_Handle_t OC_Handle, CC_Channel_t channel){

    /* Disable update event interrupt */
    Timer_Handle->pTimer->DIER &= ~(1 << TIM_DIER_UIE);

    switch(channel){
        case CHANNEL1:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x5 << TIM_CCER_CC1P_Pos,
                       OC_Handle.oc_polarity << TIM_CCER_CC1P_Pos
                      );
            /* Set capture/compare selection as output */
            CLEAR_BIT(Timer_Handle->pTimer->CCMR1, TIM_CCMR1_CC1S);
            /* Set output compare mode */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x7 << TIM_CCMR1_OC1M_Pos,
                       OC_Handle.oc_mode << TIM_CCMR1_OC1M_Pos
                      );
            /* Set initial pulse value */
            WRITE_REG(Timer_Handle->pTimer->CCR1, OC_Handle.oc_pulse);
            /* Enable preload */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x1 << TIM_CCMR1_OC1PE_Pos,
                       OC_Handle.oc_preload << TIM_CCMR1_OC1PE_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC1IE);
            /* Enable capture/compare 1 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC1E);
            break;
        case CHANNEL2:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x5 << TIM_CCER_CC2P_Pos,
                       OC_Handle.oc_polarity << TIM_CCER_CC2P_Pos
                      );
            /* Set capture/compare selection as output */
            CLEAR_BIT(Timer_Handle->pTimer->CCMR1, TIM_CCMR1_CC2S);
            /* Set output compare mode */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x7 << TIM_CCMR1_OC2M_Pos,
                       OC_Handle.oc_mode << TIM_CCMR1_OC2M_Pos
                      );
            /* Set initial pulse value */
            WRITE_REG(Timer_Handle->pTimer->CCR2, OC_Handle.oc_pulse);
            /* Enable preload */
            MODIFY_REG(Timer_Handle->pTimer->CCMR1,
                       0x1 << TIM_CCMR1_OC2PE_Pos,
                       OC_Handle.oc_preload << TIM_CCMR1_OC2PE_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC2IE);
            /* Enable capture/compare 2 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC2E);
            break;
        case CHANNEL3:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x5 << TIM_CCER_CC3P_Pos,
                       OC_Handle.oc_polarity << TIM_CCER_CC3P_Pos
                      );
            /* Set capture/compare selection as output */
            CLEAR_BIT(Timer_Handle->pTimer->CCMR2, TIM_CCMR2_CC3S);
            /* Set output compare mode */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x7 << TIM_CCMR2_OC3M_Pos,
                       OC_Handle.oc_mode << TIM_CCMR2_OC3M_Pos
                      );
            /* Set initial pulse value */
            WRITE_REG(Timer_Handle->pTimer->CCR3, OC_Handle.oc_pulse);
            /* Enable preload */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x1 << TIM_CCMR2_OC3PE_Pos,
                       OC_Handle.oc_preload << TIM_CCMR2_OC3PE_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC3IE);
            /* Enable capture/compare 3 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC3E);
            break;
        case CHANNEL4:
            /* Set polarity value */
            MODIFY_REG(Timer_Handle->pTimer->CCER,
                       0x5 << TIM_CCER_CC4P_Pos,
                       OC_Handle.oc_polarity << TIM_CCER_CC4P_Pos
                      );
            /* Set capture/compare selection as output */
            CLEAR_BIT(Timer_Handle->pTimer->CCMR2, TIM_CCMR2_CC4S);
            /* Set output compare mode */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x7 << TIM_CCMR2_OC4M_Pos,
                       OC_Handle.oc_mode << TIM_CCMR2_OC4M_Pos
                      );
            /* Set initial pulse value */
            WRITE_REG(Timer_Handle->pTimer->CCR4, OC_Handle.oc_pulse);
            /* Enable preload */
            MODIFY_REG(Timer_Handle->pTimer->CCMR2,
                       0x1 << TIM_CCMR2_OC4PE_Pos,
                       OC_Handle.oc_preload << TIM_CCMR2_OC4PE_Pos
                      );
            /* Enable interrupt */
            SET_BIT(Timer_Handle->pTimer->DIER, TIM_DIER_CC4IE);
            /* Enable capture/compare 4 channel */
            SET_BIT(Timer_Handle->pTimer->CCER, TIM_CCER_CC4E);
            break;
        default:
            break;
    }
}

void Timer_PerClkCtrl(TIM_TypeDef* pTimer , uint8_t en_or_di){

    if(en_or_di == ENABLE){
        switch((intptr_t)pTimer){
            case (intptr_t)TIM1:
                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
                break;
            case (intptr_t)TIM2:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
                break;
            case (intptr_t)TIM3:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
                break;
            case (intptr_t)TIM4:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
                break;
            case (intptr_t)TIM5:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
                break;
            case (intptr_t)TIM6:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
                break;
            case (intptr_t)TIM7:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
                break;
            case (intptr_t)TIM8:
                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
                break;
            case (intptr_t)TIM9:
                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
                break;
            case (intptr_t)TIM10:
                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN);
                break;
            case (intptr_t)TIM11:
                SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
                break;
            case (intptr_t)TIM12:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN);
                break;
            case (intptr_t)TIM13:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN);
                break;
            case (intptr_t)TIM14:
                SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
                break;
            default:
                break;
        }
    }
    else{
        switch((intptr_t)pTimer){
            case (intptr_t)TIM1:
                CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
                break;
            case (intptr_t)TIM2:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
                break;
            case (intptr_t)TIM3:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
                break;
            case (intptr_t)TIM4:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
                break;
            case (intptr_t)TIM5:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);
                break;
            case (intptr_t)TIM6:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM6EN);
                break;
            case (intptr_t)TIM7:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM7EN);
                break;
            case (intptr_t)TIM8:
                CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM8EN);
                break;
            case (intptr_t)TIM9:
                CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM9EN);
                break;
            case (intptr_t)TIM10:
                CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM10EN);
                break;
            case (intptr_t)TIM11:
                CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM11EN);
                break;
            case (intptr_t)TIM12:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM12EN);
                break;
            case (intptr_t)TIM13:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM13EN);
                break;
            case (intptr_t)TIM14:
                CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM14EN);
                break;
            default:
                break;
        }
    }
}

void Timer_IRQHandling(Timer_Handle_t* Timer_Handle){

    /* Check if TIM update interrupt happened */
    if(Timer_Handle->pTimer->SR & (1 << TIM_SR_UIF)){
        /* Clear UIF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM_SR_UIF);
        /* Call application callback */
        Timer_ApplicationEventCallback(Timer_Handle->pTimer, TIMER_UIF_EVENT);
    }

    /* Check if capture/compare interrupt happened */
    if(Timer_Handle->pTimer->SR & (1 << TIM_SR_CC1IF)){
        /* Clear CC1IF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM_SR_CC1IF);
        /* Call application callback */
        Timer_ApplicationEventCallback(Timer_Handle->pTimer, TIMER_CC1IF_EVENT);
    }
    else if(Timer_Handle->pTimer->SR & (1 << TIM_SR_CC2IF)){
        /* Clear CC2IF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM_SR_CC2IF);
        /* Call application callback */
        Timer_ApplicationEventCallback(Timer_Handle->pTimer, TIMER_CC2IF_EVENT);
    }
    else if(Timer_Handle->pTimer->SR & (1 << TIM_SR_CC3IF)){
        /* Clear CC3IF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM_SR_CC3IF);
        /* Call application callback */
        Timer_ApplicationEventCallback(Timer_Handle->pTimer, TIMER_CC3IF_EVENT);
    }
    else if(Timer_Handle->pTimer->SR & (1 << TIM_SR_CC4IF)){
        /* Clear CC4IF bit in SR register */
        Timer_Handle->pTimer->SR &= ~(1 << TIM_SR_CC4IF);
        /* Call application callback */
        Timer_ApplicationEventCallback(Timer_Handle->pTimer, TIMER_CC4IF_EVENT);
    }
    else{
        /* do nothing */
    }
}

__attribute__((weak)) void Timer_ApplicationEventCallback(TIM_TypeDef* pTimer, Timer_Event_t timer_event){

    /* This is a weak implementation. The application may override this function */
}
