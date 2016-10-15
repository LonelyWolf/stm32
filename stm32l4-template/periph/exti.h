#ifndef __EXTI_H
#define __EXTI_H


#include "stm32l4xx.h"


// Definitions of EXTI lines
#define EXTI_LINE0                EXTI_IMR1_IM0
#define EXTI_LINE1                EXTI_IMR1_IM1
#define EXTI_LINE2                EXTI_IMR1_IM2
#define EXTI_LINE3                EXTI_IMR1_IM3
#define EXTI_LINE4                EXTI_IMR1_IM4
#define EXTI_LINE5                EXTI_IMR1_IM5
#define EXTI_LINE6                EXTI_IMR1_IM6
#define EXTI_LINE7                EXTI_IMR1_IM7
#define EXTI_LINE8                EXTI_IMR1_IM8
#define EXTI_LINE9                EXTI_IMR1_IM9
#define EXTI_LINE10               EXTI_IMR1_IM10
#define EXTI_LINE11               EXTI_IMR1_IM11
#define EXTI_LINE12               EXTI_IMR1_IM12
#define EXTI_LINE13               EXTI_IMR1_IM13
#define EXTI_LINE14               EXTI_IMR1_IM14
#define EXTI_LINE15               EXTI_IMR1_IM15
#if defined(EXTI_IMR1_IM16)
#define EXTI_LINE16               EXTI_IMR1_IM16
#endif
#define EXTI_LINE17               EXTI_IMR1_IM17
#define EXTI_LINE18               EXTI_IMR1_IM18
#define EXTI_LINE19               EXTI_IMR1_IM19
#if defined(EXTI_IMR1_IM20)
#define EXTI_LINE20               EXTI_IMR1_IM20
#endif
#if defined(EXTI_IMR1_IM21)
#define EXTI_LINE21               EXTI_IMR1_IM21
#endif
#if defined(EXTI_IMR1_IM22)
#define EXTI_LINE22               EXTI_IMR1_IM22
#endif
#define EXTI_LINE23               EXTI_IMR1_IM23
#if defined(EXTI_IMR1_IM24)
#define EXTI_LINE24               EXTI_IMR1_IM24
#endif
#if defined(EXTI_IMR1_IM25)
#define EXTI_LINE25               EXTI_IMR1_IM25
#endif
#if defined(EXTI_IMR1_IM26)
#define EXTI_LINE26               EXTI_IMR1_IM26
#endif
#if defined(EXTI_IMR1_IM27)
#define EXTI_LINE27               EXTI_IMR1_IM27
#endif
#if defined(EXTI_IMR1_IM28)
#define EXTI_LINE28               EXTI_IMR1_IM28
#endif
#if defined(EXTI_IMR1_IM29)
#define EXTI_LINE29               EXTI_IMR1_IM29
#endif
#if defined(EXTI_IMR1_IM30)
#define EXTI_LINE30               EXTI_IMR1_IM30
#endif
#if defined(EXTI_IMR1_IM31)
#define EXTI_LINE31               EXTI_IMR1_IM31
#endif
#define EXTI_LINE32               EXTI_IMR2_IM32
#define EXTI_LINE33               EXTI_IMR2_IM33
#define EXTI_LINE34               EXTI_IMR2_IM34
#define EXTI_LINE35               EXTI_IMR2_IM35
#define EXTI_LINE36               EXTI_IMR2_IM36
#define EXTI_LINE37               EXTI_IMR2_IM37
#define EXTI_LINE38               EXTI_IMR2_IM38
#define EXTI_LINE39               EXTI_IMR2_IM39

// Definitions of EXTI mode configuration
#define EXTI_MODE_NONE            ((uint32_t)0x00000000U) // EXTI line disabled
#define EXTI_MODE_IRQ             ((uint32_t)0x00000001U) // EXTI line generates IRQ
#define EXTI_MODE_EVT             ((uint32_t)0x00000002U) // EXTI line generates event
#define EXTI_MODE_BOTH            ((uint32_t)0x00000003U) // EXTI line generates both IRQ and event

// Definitions of EXTI trigger configuration
#define EXTI_TRG_NONE             ((uint32_t)0x00000000U) // Trigger disabled
#define EXTI_TRG_RISING           ((uint32_t)0x00000001U) // Trigger on rising edge
#define EXTI_TRG_FALLING          ((uint32_t)0x00000002U) // Trigger on falling edge
#define EXTI_TRG_BOTH             ((uint32_t)0x00000003U) // Trigger on both falling and rising edges

// Definitions of EXTI port sources
#define EXTI_SRC_PORTA            ((uint32_t)0x00000000U) // Port A
#define EXTI_SRC_PORTB            ((uint32_t)0x00000001U) // Port B
#define EXTI_SRC_PORTC            ((uint32_t)0x00000002U) // Port C
#define EXTI_SRC_PORTD            ((uint32_t)0x00000003U) // Port D
#define EXTI_SRC_PORTE            ((uint32_t)0x00000004U) // Port E
#if defined(GPIOF)
#define EXTI_SRC_PORTF            ((uint32_t)0x00000005U) // Port F
#endif
#if defined(GPIOG)
#define EXTI_SRC_PORTG            ((uint32_t)0x00000006U) // Port G
#endif
#define EXTI_SRC_PORTH            ((uint32_t)0x00000007U) // Port H

// Definitions of EXTI pin sources
#define EXTI_PIN_SRC0             ((uint32_t)0x00000000U)
#define EXTI_PIN_SRC1             ((uint32_t)0x00000001U)
#define EXTI_PIN_SRC2             ((uint32_t)0x00000002U)
#define EXTI_PIN_SRC3             ((uint32_t)0x00000003U)
#define EXTI_PIN_SRC4             ((uint32_t)0x00000004U)
#define EXTI_PIN_SRC5             ((uint32_t)0x00000005U)
#define EXTI_PIN_SRC6             ((uint32_t)0x00000006U)
#define EXTI_PIN_SRC7             ((uint32_t)0x00000007U)
#define EXTI_PIN_SRC8             ((uint32_t)0x00000008U)
#define EXTI_PIN_SRC9             ((uint32_t)0x00000009U)
#define EXTI_PIN_SRC10            ((uint32_t)0x0000000AU)
#define EXTI_PIN_SRC11            ((uint32_t)0x0000000BU)
#define EXTI_PIN_SRC12            ((uint32_t)0x0000000CU)
#define EXTI_PIN_SRC13            ((uint32_t)0x0000000DU)
#define EXTI_PIN_SRC14            ((uint32_t)0x0000000EU)
#define EXTI_PIN_SRC15            ((uint32_t)0x0000000FU)


// Public functions and macros

// Clear EXTI line flag(s) for lines in range from 0 to 31
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[0..31] values
__STATIC_INLINE void EXTI_ClearFlag1(uint32_t EXTI_Line) {
	EXTI->PR1 = EXTI_Line;
}

// Clear EXTI line flag(s) for lines in range from 32 to 39
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[32..39] values
__STATIC_INLINE void EXTI_ClearFlag2(uint32_t EXTI_Line) {
	EXTI->PR2 = EXTI_Line;
}

// Check if the EXTI line(s) flag is set for lines in range from 0 to 31
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[0..31] values
// return: state of flag, 1 if all flags is set, 0 otherwise
__STATIC_INLINE uint32_t EXTI_IsActive1(uint32_t EXTI_Line) {
	return ((EXTI->PR1 & EXTI_Line) == EXTI_Line);
}

// Check if the EXTI line(s) flag is set for lines in range from 32 to 39
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[32..39] values
// return: state of flag, 1 if all flags is set, 0 otherwise
__STATIC_INLINE uint32_t EXTI_IsActive2(uint32_t EXTI_Line) {
	return ((EXTI->PR2 & EXTI_Line) == EXTI_Line);
}


// Function prototypes
void EXTI_cfg1(uint32_t EXTI_Line, uint32_t EXTI_mode, uint32_t EXTI_trigger);
void EXTI_cfg2(uint32_t EXTI_Line, uint32_t EXTI_mode, uint32_t EXTI_trigger);
void EXTI_src(uint32_t port_src, uint32_t pin_src);

#endif // __EXTI_H
