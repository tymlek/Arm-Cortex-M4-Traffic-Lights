// TableTrafficLight.c
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define SENSOR          (*((volatile uint32_t*)0x4002401c))
#define LIGHT_PB        (*((volatile uint32_t*)0x400050fc))
#define LIGHT_PF        (*((volatile uint32_t*)0x40025028))
	
#define goW		&FSM[0]
#define waitW 		&FSM[1]
#define goS		&FSM[2]
#define waitS		&FSM[3]
#define walk		&FSM[4]
#define blinkR1		&FSM[5]
#define blinkR2		&FSM[6]
#define blinkR3		&FSM[7]
#define blinkR4		&FSM[8]

typedef unsigned long uint32_t;

struct State {
	uint32_t OutPB;		// Output to PB5-0
	uint32_t OutPF;		// Output to PF3 and PF1
	uint32_t Time;		// Delay time
	
	const struct State* Next[8];	// An array of pointers to next states
};
typedef const struct State State_t;

const State_t FSM[9] = {
	{0x0c,0x02,1000,{goW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},
	{0x14,0x02,500,{goS,goS,goS,goS,walk,walk,goS,goS}},
	{0x21,0x02,1000,{goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},
	{0x22,0x02,500,{walk,goW,goW,goW,walk,goW,walk,walk}},
	{0x24,0x08,1000,{walk,blinkR1,blinkR1,blinkR1,blinkR1,blinkR1,blinkR1,blinkR1}},
	{0x24,0x02,500,{blinkR2,blinkR2,blinkR2,blinkR2,blinkR2,blinkR2,blinkR2,blinkR2}},
	{0x24,0x00,500,{blinkR3,blinkR3,blinkR3,blinkR3,blinkR3,blinkR3,blinkR3,blinkR3}},
	{0x24,0x02,500,{blinkR4,blinkR4,blinkR4,blinkR4,blinkR4,blinkR4,blinkR4,blinkR4}},
	{0x24,0x00,500,{goW,goW,goS,goW,goW,goW,goS,goW}}
};

State_t* state_Ptr = goW;
uint32_t Input;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****
void PortE_Init(void);
void PortB_Init(void);
void PortF_Init(void);
void SysTick_Init(void);
void SysTick_Wait(uint32_t delay);
void SysTick_Wait1ms(uint32_t delay);

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // set system clock to 80 MHz
  EnableInterrupts();
	
	SysTick_Init();
	SYSCTL_RCGC2_R |= 0x32;
	while ((SYSCTL_RCGC2_R & 0x32) != 0x32) {
	}
	
	PortE_Init();
	PortB_Init();
	PortF_Init();
	
  while(1){
    LIGHT_PB = state_Ptr->OutPB;
		LIGHT_PF = state_Ptr->OutPF;
		SysTick_Wait1ms(state_Ptr->Time);
		Input = SENSOR;
		state_Ptr = state_Ptr->Next[Input];
  }
}

void SysTick_Init(void) {
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_CTRL_R = 5;
}

void SysTick_Wait(uint32_t delay) {
	NVIC_ST_RELOAD_R = delay-1;
	NVIC_ST_CURRENT_R = 0;
	while (!(NVIC_ST_CTRL_R&0x00010000))
	{
	}
}

void SysTick_Wait1ms(uint32_t delay) {
	uint32_t i;
	for (i = 0; i<delay; ++i)
		SysTick_Wait(80000);
}

void PortE_Init(void) {
	GPIO_PORTE_AMSEL_R &= ~0x07;
	GPIO_PORTE_PCTL_R &= ~0x00000FFF;
	GPIO_PORTE_DIR_R &= ~0x07;
	GPIO_PORTE_AFSEL_R &= ~0x07;
	GPIO_PORTE_DEN_R |= 0x07;
}

void PortB_Init(void) {
	GPIO_PORTB_AMSEL_R &= ~0x3F;
	GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;
	GPIO_PORTB_DIR_R |= 0x3F;
	GPIO_PORTB_AFSEL_R &= ~0x3F;
	GPIO_PORTB_DEN_R |= 0x3F;
}

void PortF_Init(void) {
	GPIO_PORTF_AMSEL_R &= ~0x0A;
	GPIO_PORTF_PCTL_R &= ~0x0000F0F0;
	GPIO_PORTF_DIR_R |= 0x0A;
	GPIO_PORTF_AFSEL_R &= ~0x0A;
	GPIO_PORTF_DEN_R |= 0x0A;
}
