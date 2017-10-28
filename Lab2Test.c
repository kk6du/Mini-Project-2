// Lab2Test.c
// Runs on LM4F120/TM4C123
// Real Time Operating System for Labs 2 and 3

// Jonathan W. Valvano 2/20/17, valvano@mail.utexas.edu
// Modified by Sile Shu 10/4/17, ss5de@virginia.edu
// You may use, edit, run or distribute this file
// You are free to change the syntax/organization of this file

#include <stdint.h>
#include "OS.h"
#include "tm4c123gh6pm.h"
#include "LCD.h"
#include <string.h>
#include "UART.h"

#define PE0  (*((volatile unsigned long *)0x40024004))
#define PE1  (*((volatile unsigned long *)0x40024008))
#define PE2  (*((volatile unsigned long *)0x40024010))
#define PE3  (*((volatile unsigned long *)0x40024020))
#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units

//user defined types for MiniProject2
unsigned int TimeSlice = 0; //amt of time to run
unsigned int ContextSwitchTime = 0; //amt of time to switch bt tasks
//call os_time before and after event to get time slice
//call os_time at end of task and start of next one to get contextswitchtime
//use os_timedifference for these

unsigned long NumCreated;   // number of foreground threads created

void PortE_Init(void){ unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x10;       // activate port E
  delay = SYSCTL_RCGCGPIO_R;
  delay = SYSCTL_RCGCGPIO_R;
  GPIO_PORTE_DIR_R |= 0x0F;    // make PE3-0 output heartbeats
  GPIO_PORTE_AFSEL_R &= ~0x0F;   // disable alt funct on PE3-0
  GPIO_PORTE_DEN_R |= 0x0F;     // enable digital I/O on PE3-0
  GPIO_PORTE_PCTL_R = ~0x0000FFFF;
  GPIO_PORTE_AMSEL_R &= ~0x0F;;      // disable analog functionality on PF
}

//******************* Measurement of context switch time**********
// Run this to measure the time it takes to perform a task switch
// UART0 not needed
// SYSTICK interrupts, period established by OS_Launch
// first timer not needed
// second timer not needed
// SW1 not needed,
// SW2 not needed
// logic analyzer on PF1 for systick interrupt (in your OS)
//                on PE0 to measure context switch time
void Thread8(void){       // only thread running
  while(1){
    PE0 ^= 0x01;      // debugging profile
  }
}

int Testmain0(void){       // Testmain7
  PortE_Init();
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread8,128,2);
  OS_Launch(TIME_1MS/10); // 100us, doesn't return, interrupts enabled in here
  return 0;             // this never executes
}

//*******************Initial TEST**********
// This is the simplest configuration, test this first, (Lab 2 part 1)
// run this with
// no UART interrupts
// no SYSTICK interrupts
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores
unsigned long Count1;   // number of times thread1 loops
unsigned long Count2;   // number of times thread2 loops
unsigned long Count3;   // number of times thread3 loops
unsigned long Count4;   // number of times thread4 loops
unsigned long Count5;   // number of times thread5 loops

unsigned char timeflag;

unsigned long startTimeContext;
unsigned long endTimeContext;
unsigned long startTimeSlice;
unsigned long endTimeSlice;
void Thread1(void){
  Count1 = 0;
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
		startTimeContext = OS_Time(); //flag for context switching
		timeflag = 1;
    OS_Suspend();      // cooperative multitasking
  }
}

void Thread2(void){
	if (timeflag == 1)
	{
		endTimeContext = OS_Time();
		timeflag = 0;
		startTimeSlice = OS_Time();
	}
  Count2 = 0;
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count2++;
		endTimeSlice = OS_Time();
    OS_Suspend();      // cooperative multitasking
  }
	//endTimeSlice = OS_Time();
}
void Thread3(void){ //find time difference between the first two threads
	ContextSwitchTime = OS_TimeDifference(startTimeContext, endTimeContext);
	TimeSlice = OS_TimeDifference(startTimeSlice, endTimeSlice);
  Count3 = 0;
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count3++;
    OS_Suspend();      // cooperative multitasking
  }
}

int Testmain1(void){  // Testmain1
  OS_Init();          // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1,128,1);
  NumCreated += OS_AddThread(&Thread2,128,2);
  NumCreated += OS_AddThread(&Thread3,128,3);
  // Count1 Count2 Count3 should be equal or off by one at all times
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Second TEST**********
// Once the initalize test runs, test this (Lab 2 part 1)
// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// no timer interrupts
// no switch interrupts
// no ADC serial port or LCD output
// no calls to semaphores

unsigned long startTimeContextb;
unsigned long endTimeContextb;
unsigned long startTimeSliceb;
unsigned long endTimeSliceb;
unsigned long timeflagb;
void Thread1b(void){
  Count1 = 0;
  for(;;){
    PE0 ^= 0x01;       // heartbeat
    Count1++;
		startTimeContextb = OS_Time(); //flag for context switching
		timeflagb = 1;
  }
}
void Thread2b(void){
	if (timeflagb == 1)
	{
		endTimeContextb = OS_Time();
		timeflagb = 0;
		startTimeSliceb = OS_Time();
	}
  Count2 = 0;
  for(;;){
    PE1 ^= 0x02;       // heartbeat
    Count2++;
		endTimeSliceb = OS_Time();
  }
}
void Thread3b(void){
	ContextSwitchTime = OS_TimeDifference(startTimeContextb, endTimeContextb);
	TimeSlice = OS_TimeDifference(startTimeSliceb, endTimeSliceb);
  Count3 = 0;
  for(;;){
    PE2 ^= 0x04;       // heartbeat
    Count3++;
  }
}
int Testmain2(void){  // Testmain2
  OS_Init();           // initialize, disable interrupts
  PortE_Init();       // profile user threads
  NumCreated = 0 ;
  NumCreated += OS_AddThread(&Thread1b,128,1);
  NumCreated += OS_AddThread(&Thread2b,128,2);
  NumCreated += OS_AddThread(&Thread3b,128,3);
  // Count1 Count2 Count3 should be equal on average
  // counts are larger than testmain1

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Third TEST**********
// Once the second test runs, test this (Lab 2 part 2)
// no UART1 interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// PortF GPIO interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyc;        // set in background
int Lost;
void BackgroundThread1c(void){   // called at 1000 Hz
  Count1++;
  OS_Signal(&Readyc);
}
void Thread5c(void){
  for(;;){
    OS_Wait(&Readyc);
    Count5++;   // Count2 + Count5 should equal Count1
    Lost = Count1-Count5-Count2;
  }
}
void Thread2c(void){
  OS_InitSemaphore(&Readyc,0);
  Count1 = 0;    // number of times signal is called
  Count2 = 0;
  Count5 = 0;    // Count2 + Count5 should equal Count1
  NumCreated += OS_AddThread(&Thread5c,128,3);
  OS_AddPeriodicThread(&BackgroundThread1c,TIME_1MS,0);
  for(;;){
    OS_Wait(&Readyc);
    Count2++;   // Count2 + Count5 should equal Count1
  }
}

void Thread3c(void){
  Count3 = 0;
  for(;;){
    Count3++;
  }
}
void Thread4c(void){ int i;
  for(i=0;i<64;i++){
    Count4++;
    OS_Sleep(10);
  }
  OS_Kill();
  Count4 = 0;
}
void BackgroundThread5c(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4c,128,3);
}

int Testmain3(void){   // Testmain3
  Count4 = 0;
  OS_Init();           // initialize, disable interrupts
// Count2 + Count5 should equal Count1
  NumCreated = 0 ;
  OS_AddSW1Task(&BackgroundThread5c,2);
  NumCreated += OS_AddThread(&Thread2c,128,2);
  NumCreated += OS_AddThread(&Thread3c,128,3);
  NumCreated += OS_AddThread(&Thread4c,128,3);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

//*******************Fourth TEST**********
// Once the third test runs, run this example (Lab 2 part 2)
// Count1 should exactly equal Count2
// Count3 should be very large
// Count4 increases by 640 every time select is pressed
// NumCreated increase by 1 every time select is pressed

// no UART interrupts
// SYSTICK interrupts, with or without period established by OS_Launch
// Timer interrupts, with or without period established by OS_AddPeriodicThread
// Select switch interrupts, active low
// no ADC serial port or LCD output
// tests the spinlock semaphores, tests Sleep and Kill
Sema4Type Readyd;        // set in background
void BackgroundThread1d(void){   // called at 1000 Hz
static int i=0;
  i++;
  if(i==50){
    i = 0;         //every 50 ms
    Count1++;
    OS_bSignal(&Readyd);
  }
}
void Thread2d(void){
  OS_InitSemaphore(&Readyd,0);
  Count1 = 0;
  Count2 = 0;
  for(;;){
    OS_bWait(&Readyd);
    Count2++;
  }
}
void Thread3d(void){
  Count3 = 0;
  for(;;){
    Count3++;
  }
}
void Thread4d(void){ int i;
  for(i=0;i<640;i++){
    Count4++;
    OS_Sleep(1);
  }
  OS_Kill();
}
void BackgroundThread5d(void){   // called when Select button pushed
  NumCreated += OS_AddThread(&Thread4d,128,3);
}
int main(void){   // Testmain4
  Count4 = 0;
  OS_Init();           // initialize, disable interrupts
  NumCreated = 0 ;
  OS_AddPeriodicThread(&BackgroundThread1d,PERIOD,0);
  OS_AddSW1Task(&BackgroundThread5d,2);
  NumCreated += OS_AddThread(&Thread2d,128,2);
  NumCreated += OS_AddThread(&Thread3d,128,3);
  NumCreated += OS_AddThread(&Thread4d,128,3);
  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
