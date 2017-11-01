// MODIFIED BY SILE SHU 2017.6
// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

   Programs 4.4 through 4.12, section 4.2

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include <stdint.h>
#include "os.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include "UART.h"
#include "joystick.h"

#define NULL (void*) 0

#define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

// Additional defines for Lab 2
#define NVIC_EN0_INT21          0x00200000  // Interrupt 21 enable
#define NVIC_EN1_INT35					0x00000008

#define TIMER_CFG_32_BIT_TIMER  0x00000000  // 32-bit timer configuration
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_ICR_TATOCINT      0x00000001  // GPTM TimerA Time-Out Raw
                                            // Interrupt
#define TIMER_TAILR_M           0xFFFFFFFF  // GPTM Timer A Interval Load
                                            // Register

#define GPIO_PORTF2             (*((volatile uint32_t *)0x40025010))

// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void StartOS(void);


#define NUMTHREADS  20       // maximum number of threads
#define STACKSIZE   100      // number of 32-bit words in stack

struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
  uint32_t sleepCt;	 // sleep counter in MS
  uint32_t id;       // thread #
	uint32_t available; // used to indicate if this tcb is available or not
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];
tcbType *head = &tcbs[0];


// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 80 MHz PLL
// input:  none
// output: none
void OS_Init(void){int i;
  OS_DisableInterrupts();
  PLL_Init(Bus80MHz);                 // set processor clock to 80 MHz
	InitTimer2A(TIME_1MS);  // initialize Timer2A which is used for software timer and decrease the sleepCt
	InitTimer3A();
  OS_ClearMsTime();

  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
															// lowest PRI so only foreground interrupted
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7

	head = &tcbs[0];

	//everything is initialized to available
	for (i = 0; i < NUMTHREADS; i++)
	{
		tcbs[i].available = 1;
	}
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}


//******** OS_AddThreads ***************
// Test function that adds three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OS_AddThreads(void(*task0)(void),
                 void(*task1)(void),
                 void(*task2)(void)){ int32_t status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);
  return 1;               // successful
}

//******** OS_AddThread ***************
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
static uint32_t ThreadNum = 0;
int OS_AddThread(void(*task)(void),
   unsigned long stackSize, unsigned long priority)
{
  // Your code here.
	unsigned	char	i,j;
	int32_t	status,thread;
	 	 status	=	StartCritical();
	if	(ThreadNum	==	0)
	{	 	 //	start	add	thread
		tcbs[0].available	=	0;
		tcbs[0].next	=	&tcbs[0];	 	 //	first,	create	a	single	cycle
		RunPt	=	&tcbs[0];	//start	from	tcbs[0]
		thread	=	0;
	}
	else
	{	 	 	 //	not	the	start
		for	(i=0;i<NUMTHREADS;i++){
		if	(tcbs[i].available)	break;	 	 	 //	find	an	available	tcb	for	the	new	thread
		}
		thread	=	i;
		tcbs[i].available	=	0;	//	make	this	tcb	no	longer	available
		for	(j	=	(thread	+	NUMTHREADS	- 1)%NUMTHREADS;	j	!=	thread;	j	=	(j	+	NUMTHREADS	- 1)%NUMTHREADS)
		{
			if	(tcbs[j].available	==	0)	break;	//	find	a	previous	tcb	which	has	been	used
		}
		//	add	this	tcb	into	the	link	list	cycle
		tcbs[thread].next	=	tcbs[j].next;
		tcbs[j].next	=	&tcbs[thread];
	}
	tcbs[thread].id	=	thread;
	SetInitialStack(thread);
	Stacks[thread][STACKSIZE-2]	=	(int32_t)(task);	//	PC
	ThreadNum++;
	EndCritical(status);
	return	1;
}

//******** OS_Id ***************
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero
unsigned long OS_Id(void) {
	// Your code here.
	return RunPt->id;
}


// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime){
	RunPt->sleepCt = sleepTime;
	OS_Suspend();
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void){
	// Your code here.
	unsigned	char	i;
	int32_t	thread;
	RunPt->available	=	1;
	thread	=	OS_Id();
	for	 (i =(thread + NUMTHREADS	- 1)%NUMTHREADS;i != thread; i = (i + NUMTHREADS-1)%NUMTHREADS)
	{
		if(tcbs[i].available	==	0)	break;	 	 	 //	find	the	previous	used	tcb
	}
	ThreadNum--;
	tcbs[i].next	=	tcbs[thread].next;
	OS_Suspend();	//	switch	the	thread
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) {
 	// Your code here.
	NVIC_ST_CURRENT_R = 0; //reset counter
	NVIC_INT_CTRL_R	=	0x04000000; //trigger systick
}

// ******** OS_InitSemaphore ************
// initialize semaphore
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long value){
	// Your code here.
	semaPt->Value = value;
}

// ******** OS_Wait ************
// decrement semaphore
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt){
	// Your code here.
	OS_DisableInterrupts();
	while(semaPt->Value == 0)
	{
		OS_EnableInterrupts();
		OS_DisableInterrupts();
	}
	(semaPt->Value) = (semaPt->Value)-1;
	OS_EnableInterrupts();
}

// ******** OS_Signal ************
// increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt){
	// Your code here.
	OS_DisableInterrupts();
	semaPt->Value = semaPt->Value + 1;
	OS_EnableInterrupts();
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt){
	// Your code here.
	OS_DisableInterrupts();
	semaPt->Value = 0; //Lab2 set to 0
	OS_EnableInterrupts();
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt){
	// Your code here.
	OS_DisableInterrupts();
	semaPt->Value = 1;
	OS_EnableInterrupts();
}

void (*PeriodicTask)(void);
//******** OS_AddPeriodicThread ***************
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void),
   unsigned long period, unsigned long priority) {
	PeriodicTask = task;
	InitTimer1A(period);
  return 1;
}

void (*ButtonOneTask)(void);
//******** OS_AddSW1Task ***************
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority) {
	// Your code here.
	//s1 maps to pd6 which is the button that we need to check
	ButtonOneTask = task; //this is called in the push button interrupt handler
	ButtonOneInit();
	return 1;
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 20ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(unsigned long theTimeSlice){
	NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  	NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  	StartOS();                   // start on the first task
}

///******** Scheduler ***************
// The scheduler
// Inputs: none
// Outputs: none (does not return)
void Scheduler(void){
	// Your code here.
	RunPt = RunPt->next; //skip one
	while(RunPt->sleepCt)
	{
		RunPt=RunPt->next;
	}
}

// ******** OS_Time ************
// return the system time
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as
//   this function and OS_TimeDifference have the same resolution and precision
unsigned long OS_Time(void) {
	// Your code here.
	return	TIMER3_TAILR_R	- TIMER3_TAV_R;
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as
//   this function and OS_Time have the same resolution and precision
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	// Your code here.

	return (stop - start);
}

// Ms time system
static uint32_t MSTime;
// ******** OS_ClearMsTime ************
// sets the system time to zero
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void) {
	// Your code here.
	MSTime = 0;
}

// ******** OS_MsTime ************
// reads the current time in msec
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void) {
	// Your code here.
	return MSTime;
}

void InitTimer1A(unsigned long period) {
	long sr;
	volatile unsigned long delay;

	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x02;

  while((SYSCTL_RCGCTIMER_R & 0x02) == 0){} // allow time for clock to stabilize

  TIMER1_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER1_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;
  TIMER1_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|(3 << 13);	//3
  NVIC_EN0_R = NVIC_EN0_INT21;     // 8) enable interrupt 21 in NVIC
  TIMER1_TAPR_R = 0;
  TIMER1_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A

  EndCritical(sr);
}

void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
	(*PeriodicTask)();
}

void InitTimer2A(unsigned long period) {
	long sr;
<<<<<<< HEAD
	volatile unsigned long delay;

	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x04;

  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;

=======
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x04;
	
    while((SYSCTL_RCGCTIMER_R & 0x04) == 0){} // allow time for clock to stabilize
	
>>>>>>> 5074ebdbb957ba3815329985066e8ba6e12a22e2
  TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER2_TAILR_R = period - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER2_ICR_R = TIMER_ICR_TATOCINT;
  TIMER2_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 31-29 for timer2A
  NVIC_PRI5_R = (NVIC_PRI5_R&0x00FFFFFF)|(2 << 29);
  NVIC_EN0_R = NVIC_EN0_INT23;     // 8) enable interrupt 23 in NVIC
  TIMER2_TAPR_R = 0;
  TIMER2_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer2A

  EndCritical(sr);
}

void Timer2A_Handler(void){
	TIMER2_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer2A timeout
	// Your code here.
	if (RunPt->sleepCt != 0)
	{
		RunPt->sleepCt = RunPt->sleepCt - 1;
	}
}

void InitTimer3A(void) {
	long sr;
<<<<<<< HEAD
	volatile unsigned long delay;

	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;

  delay = SYSCTL_RCGCTIMER_R;
	delay = SYSCTL_RCGCTIMER_R;

=======
	
	sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;
	
  while((SYSCTL_RCGCTIMER_R & 0x08) == 0){} // allow time for clock to stabilize
	
>>>>>>> 5074ebdbb957ba3815329985066e8ba6e12a22e2
  TIMER3_CTL_R &= ~TIMER_CTL_TAEN; // 1) disable timer1A during setup
                                   // 2) configure for 32-bit timer mode
  TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;
                                   // 3) configure for periodic mode, default down-count settings
  TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER3_TAILR_R = 0xFFFFFFFF - 1;     // 4) reload value
                                   // 5) clear timer1A timeout flag
  TIMER3_ICR_R = TIMER_ICR_TATOCINT;
  TIMER3_IMR_R |= TIMER_IMR_TATOIM;// 6) arm timeout interrupt
								   // 7) priority shifted to bits 15-13 for timer1A
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|(1 << 29);	//1
  NVIC_EN1_R = NVIC_EN1_INT35;     // 8) enable interrupt 21 in NVIC
  TIMER3_TAPR_R = 0;
  TIMER3_CTL_R |= TIMER_CTL_TAEN;  // 9) enable timer1A

  EndCritical(sr);
}

void Timer3A_Handler(void){

  TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer1A timeout
}

// SWtask part
#define BUTTON1   (*((volatile uint32_t *)0x40007100))  /* PD6 */
volatile static uint32_t Last;

//void (*ButtonOneTask)(void);
void ButtonOneInit(void){
  SYSCTL_RCGCGPIO_R |= 0x00000008; // 1) activate clock for Port D
  while((SYSCTL_PRGPIO_R&0x08) == 0){};// allow time for clock to stabilize
	                                 // 2) no need to unlock PD6
		//added lines to fix error
	GPIO_PORTD_LOCK_R = 0x4C4F434B;  // unlock PD6
  GPIO_PORTD_CR_R = 0x40;          // allow changes

  GPIO_PORTD_AMSEL_R &= ~0x40;     // 3) disable analog on PD6
                                   // 4) configure PD6 as GPIO
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xF0FFFFFF)+0x00000000;
  GPIO_PORTD_DIR_R &= ~0x40;       // 5) make PD6 input
  GPIO_PORTD_AFSEL_R &= ~0x40;     // 6) disable alt funct on PD6
	GPIO_PORTD_DEN_R |= 0x40;        // 7) enable digital I/O on PD6
	GPIO_PORTD_PUR_R |= 0x40;     //     enable weak pull-up on PD6
  GPIO_PORTD_IS_R &= ~0x40;     // (d) PD6 is edge-sensitive
  GPIO_PORTD_IBE_R |= 0x40;     //     PD6 is both edges
	GPIO_PORTD_ICR_R = 0x40;      // (e) clear flag6
  GPIO_PORTD_IM_R |= 0x40;      // (f) arm interrupt on PD6 *** No IME bit as mentioned in Book ***

  NVIC_PRI0_R = (NVIC_PRI0_R&0x1FFFFFFF)|0x40000000; // (g) priority 2
  NVIC_EN0_R = 0x00000008;      // (h) enable interrupt 3 in NVIC
	Last = BUTTON1;
}

void static DebouncePD6(void) {
  OS_Sleep(10);      //foreground sleep
  Last = BUTTON1;
  GPIO_PORTD_ICR_R = 0x40;
  GPIO_PORTD_IM_R |= 0x40;
  OS_Kill();
}
void GPIOPortD_Handler(void) {  // called on touch of either SW1 or SW2
	GPIO_PORTD_IM_R &= ~0x40;  //disarm interrupt on PD6
	if (Last){
		(*ButtonOneTask)();
	}
	OS_AddThread(DebouncePD6,128,2);
}
