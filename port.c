/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the CCore (MCore) port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "regdef.h"

/* Constants required to manipulate the NVIC. */
#define portNVIC_SYSTICK_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe0001000 ) )    //ARM M0: 0xe000e010
#define portNVIC_SYSTICK_LOAD_REG           ( * ( ( volatile uint32_t * ) 0xe0001004 ) )    //ARM M0: 0xe000e014
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  ( * ( ( volatile uint32_t * ) 0xe0001008 ) )    //ARM M0: 0xe000e018
#define portNVIC_INT_CTRL_REG               ( * ( ( volatile uint32_t * ) 0xe0000000 ) )    //ARM M0: 0xe000ed04
#define portNVIC_SYSPRI2_REG                ( * ( ( volatile uint32_t * ) 0xe0000060 ) )    //ARM M0: 0xe000ed20
#define portNVIC_SYSTICK_CLK_BIT            ( 1UL << 2UL )
#define portNVIC_SYSTICK_INT_BIT            ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT         ( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT     ( 1UL << 16UL )
#define portNVIC_PENDSVSET_BIT              ( 1UL << 28UL )
#define portMIN_INTERRUPT_PRIORITY          ( 3UL )
#define portNVIC_PENDSV_PRI                 ( portMIN_INTERRUPT_PRIORITY << 16UL )
#define portNVIC_SYSTICK_PRI                ( portMIN_INTERRUPT_PRIORITY << 24UL )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR            ( 0x80000140 )      // ARM M0: 0x01000000

/* The systick is a 24-bit counter. */
#define portMAX_24_BIT_NUMBER                ( 0xffffffUL )

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#ifndef portMISSED_COUNTS_FACTOR
    #define portMISSED_COUNTS_FACTOR            ( 45UL )
#endif

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );

/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void )    __attribute__ (( naked ));
void IntDefaultHandler( void );

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void vPortStartFirstTask( void ) __attribute__ (( naked ));

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*-----------------------------------------------------------*/

/*
* The number of SysTick increments that make up one tick period.
*/
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook (void)
{
    for(;;){
        // volatile uint8_t malloc_failed = 1;
    }
}


void vApplicationStackOverflowHook(void)
{
    for(;;){
        // volatile uint8_t stack_overflow_failed = 1;
    }
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    /* Simulate the stack frame as it would be created by a context switch
    interrupt. */
    // ARM M0: pxTopOfStack--;                                             /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
    // ARM M0: *pxTopOfStack = portINITIAL_XPSR;                           /* xPSR */
    // ARM M0: pxTopOfStack--;
    // ARM M0: *pxTopOfStack = ( StackType_t ) pxCode;                     /* PC */
    // ARM M0: pxTopOfStack--;
    // ARM M0: *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;    /* LR */
    // ARM M0: pxTopOfStack -= 5;                                          /* R12, R3, R2 and R1. */
    // ARM M0: *pxTopOfStack = ( StackType_t ) pvParameters;               /* R0 */
    // ARM M0: pxTopOfStack -= 8;                                          /* R11..R4. */

#if 0
    pxTopOfStack--;                                             /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;    /* R15 */
    pxTopOfStack-=6;                                            /* R3, R4, R5, R6, R7 */
    *pxTopOfStack = ( StackType_t ) pvParameters;               /*R2*/
    pxTopOfStack-=2;                                            /*R1*/
    *pxTopOfStack = portINITIAL_XPSR;                           /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode;                     /* PC */
    pxTopOfStack -= 8;                                          /* R14..R8. */
#else
    pxTopOfStack--; /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts. */
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;    /* R15 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000007;                                 /*R7*/
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000006;                                 /*R6*/
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000005;                                 /*R5*/
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000004;                                 /*R4*/
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000003;                                 /*R3*/
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pvParameters;               /*R2*/
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000001;                                 /*R1*/
    pxTopOfStack--;
    *pxTopOfStack = portINITIAL_XPSR;                           /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) pxCode;                     /* PC */
    pxTopOfStack-=2;                                            /* R15 */
    *pxTopOfStack = 0xC0000014;                                 /* R14 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000013;                                 /* R13 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000012;                                 /* R12 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000011;                                 /* R11 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000010;                                 /* R10 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000009;                                 /* R9 */
    pxTopOfStack--;
    *pxTopOfStack = 0xC0000008;                                 /* R8 */
#endif
    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    volatile uint32_t ulDummy = 0UL;

    /* A function that implements a task must not exit or attempt to return to
    its caller as there is nothing to return to.  If a task wants to exit it
    should instead call vTaskDelete( NULL ).

    Artificially force an assert() to be triggered if configASSERT() is
    defined, then stop here so application writers can catch the error. */
    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();
    while( ulDummy == 0 )
    {
        /* This file calls prvTaskExitError() after the scheduler has been
        started to remove a compiler warning about the function being defined
        but never called.  ulDummy is used purely to quieten other warnings
        about code appearing after this function is called - making ulDummy
        volatile makes the compiler think the function could return and
        therefore not output an 'unreachable code' warning for code that appears
        after it. */
    }
}
/*-----------------------------------------------------------*/

void vPortSVCHandler( void )
{
    /* This function is no longer used, but retained for backward
    compatibility. */
}
/*-----------------------------------------------------------*/

void IntDefaultHandler(void)
{
    for(;;){}
}
/*-----------------------------------------------------------*/

void vPortStartFirstTask( void )
{
    /* The MSP stack is not reset as, unlike on M3/4 parts, there is no vector
    table offset register that can be used to locate the initial stack value.
    Not all M0 parts have the application vector table at address 0. */
    
    // ARM M0: __asm volatile(
    // ARM M0: "    .syntax unified                \n"
    // ARM M0: "    ldr  r2, pxCurrentTCBConst2    \n" /* Obtain location of pxCurrentTCB. */
    // ARM M0: "    ldr  r3, [r2]                  \n"
    // ARM M0: "    ldr  r0, [r3]                  \n" /* The first item in pxCurrentTCB is the task top of stack. */
    // ARM M0: "    adds r0, #32                   \n" /* Discard everything up to r0. */
    // ARM M0: "    msr  psp, r0                   \n" /* This is now the new top of stack to use in the task. */
    // ARM M0: "    movs r0, #2                    \n" /* Switch to the psp stack. */
    // ARM M0: "    msr  CONTROL, r0               \n"
    // ARM M0: "    isb                            \n"
    // ARM M0: "    pop  {r0-r5}                   \n" /* Pop the registers that are saved automatically. */
    // ARM M0: "    mov  lr, r5                    \n" /* lr is now in r5. */
    // ARM M0: "    pop  {r3}                      \n" /* Return address is now in r3. */
    // ARM M0: "    pop  {r2}                      \n" /* Pop and discard XPSR. */
    // ARM M0: "    cpsie i                        \n" /* The first task has its context and interrupts can be enabled. */
    // ARM M0: "    bx   r3                        \n" /* Finally, jump to the user defined task code. */
    // ARM M0: "                                   \n"
    // ARM M0: "    .align 4                       \n"
    // ARM M0: "pxCurrentTCBConst2: .word pxCurrentTCB      "
    // ARM M0:               );
    
    __asm volatile(
    "psrclr ie                      \n"
    "lrw r2, pxCurrentTCBConst2     \n"     /* Obtain location of pxCurrentTCB. */
    "ld.w r2, (r2, 0)               \n"
    "ld.w r2, (r2, 0)               \n"
    "ld.w r2, (r2, 0)               \n"     /* The first item in pxCurrentTCB is the task top of stack. */
    "                               \n"
    "mov r0, r2                     \n"
    "ldm r8-r15, (r0)               \n"
    "addi r0, 32                    \n"     /* This is now the new top of stack to use in the task. */
    "                               \n"
    "psrset ie                      \n"
    "rte                            \n"
    "                               \n"
    ".align 4                       \n"
    "pxCurrentTCBConst2: .long pxCurrentTCB"
   );
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
    /* Make PendSV, CallSV and SysTick the same priority as the kernel. */
    portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
    portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

    /* Start the timer that generates the tick ISR.  Interrupts are disabled
    here already. */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    vPortStartFirstTask();

    /* Should never get here as the tasks will now be executing!  Call the task
    exit error function to prevent compiler warnings about a static function
    not being called in the case that the application writer overrides this
    functionality by defining configTASK_RETURN_ADDRESS.  Call
    vTaskSwitchContext() so link time optimisation does not remove the
    symbol. */
    vTaskSwitchContext();
    prvTaskExitError();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /* Not implemented in ports where there is nothing to return to.
    Artificially force an assert. */
    configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortYield( void )
{
    /* Set a PendSV to request a context switch. */
    portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;

    /* Barriers are normally not required but do ensure the code is completely
    within the specified behaviour for the architecture. */
    //ARM M0: __asm volatile( "dsb" ::: "memory" );
    //ARM M0: __asm volatile( "isb" );
    
    __asm volatile("sync" ::: "memory");
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;
    //ARM M0: __asm volatile( "dsb" ::: "memory" );
    //ARM M0: __asm volatile( "isb" );
    __asm volatile("sync" ::: "memory");
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}
/*-----------------------------------------------------------*/

uint32_t ulSetInterruptMaskFromISR( void )
{
    /*
    __asm volatile(
                    " mrs r0, PRIMASK    \n"
                    " cpsid i            \n"
                    " bx lr                  "
                    ::: "memory"
                  );
    */
    /*__asm volatile(
     *               " movi    r2, 0x01   \n"
                     " psrclr ie          \n"
                     " jmp r15              "
                     ::: "memory"
                  );
    */
	return 0;
}
/*-----------------------------------------------------------*/

void vClearInterruptMaskFromISR( __attribute__( ( unused ) ) uint32_t ulMask )
{
    /*
    __asm volatile(
                    " msr PRIMASK, r0    \n"
                    " bx lr                  "
                    ::: "memory"
                  );
    */
    /*__asm volatile("psrclr ie      \n"
                   "jmp r15          "
                   ::: "memory");
    */
}
/*-----------------------------------------------------------*/

void xPortPendSVHandler( void )
{
    /* This is a naked function. */

    // ARM M0: __asm volatile
    // ARM M0: (
    // ARM M0: "    .syntax unified                        \n"
    // ARM M0: "    mrs r0, psp                            \n"
    // ARM M0: "                                        \n"
    // ARM M0: "    ldr    r3, pxCurrentTCBConst            \n" /* Get the location of the current TCB. */
    // ARM M0: "    ldr    r2, [r3]                        \n"
    // ARM M0: "                                        \n"
    // ARM M0: "    subs r0, r0, #32                    \n" /* Make space for the remaining low registers. */
    // ARM M0: "    str r0, [r2]                        \n" /* Save the new top of stack. */
    // ARM M0: "    stmia r0!, {r4-r7}                    \n" /* Store the low registers that are not saved automatically. */
    // ARM M0: "     mov r4, r8                            \n" /* Store the high registers. */
    // ARM M0: "     mov r5, r9                            \n"
    // ARM M0: "     mov r6, r10                            \n"
    // ARM M0: "     mov r7, r11                            \n"
    // ARM M0: "     stmia r0!, {r4-r7}                    \n"
    // ARM M0: "                                        \n"
    // ARM M0: "    push {r3, r14}                        \n"
    // ARM M0: "    cpsid i                                \n"
    // ARM M0: "    bl vTaskSwitchContext                \n"
    // ARM M0: "    cpsie i                                \n"
    // ARM M0: "    pop {r2, r3}                        \n" /* lr goes in r3. r2 now holds tcb pointer. */
    // ARM M0: "                                        \n"
    // ARM M0: "    ldr r1, [r2]                        \n"
    // ARM M0: "    ldr r0, [r1]                        \n" /* The first item in pxCurrentTCB is the task top of stack. */
    // ARM M0: "    adds r0, r0, #16                    \n" /* Move to the high registers. */
    // ARM M0: "    ldmia r0!, {r4-r7}                    \n" /* Pop the high registers. */
    // ARM M0: "     mov r8, r4                            \n"
    // ARM M0: "     mov r9, r5                            \n"
    // ARM M0: "     mov r10, r6                            \n"
    // ARM M0: "     mov r11, r7                            \n"
    // ARM M0: "                                        \n"
    // ARM M0: "    msr psp, r0                            \n" /* Remember the new top of stack for the task. */
    // ARM M0: "                                        \n"
    // ARM M0: "    subs r0, r0, #32                    \n" /* Go back for the low registers that are not automatically restored. */
    // ARM M0: "     ldmia r0!, {r4-r7}                    \n" /* Pop low registers.  */
    // ARM M0: "                                        \n"
    // ARM M0: "    bx r3                                \n"
    // ARM M0: "                                        \n"
    // ARM M0: "    .align 4                            \n"
    // ARM M0: "pxCurrentTCBConst: .word pxCurrentTCB      "
    // ARM M0: );

    __asm volatile
    (
    "    psrclr ie                           \n"
    "    lrw r3, pxCurrentTCBConst           \n" // Get the location of the current TCB.
    "    ld.w r2, (r3, 0)                    \n"
    "    ld.w r2, (r2, 0)                    \n"
    "                                        \n"
    "    subi r0, 32                         \n" // r8..r15
    "    st.w r0, (r2, 0)                    \n"
    "                                        \n"
    "    stm r8-r15, (r0)                    \n"
    "    jsri vTaskSwitchContext             \n"
    "                                        \n"
    "   lrw r3, pxCurrentTCBConst            \n"
    "   ld.w r2, (r3, 0)                     \n"
    "   ld.w r2, (r2, 0)                     \n"
    "   ld.w r0, (r2, 0)                     \n"
    "                                        \n"
    "   ldm r8-r15, (r0)                     \n"
    "   addi r0, 32                          \n"
    "                                        \n"
    "   psrset ie                            \n"
    "   rte                                  \n"
    "                                        \n"
    "   .align 4                             \n"
    "pxCurrentTCBConst: .long pxCurrentTCB     "
    );

}
/*-----------------------------------------------------------*/

void xPortSysTickHandler( void )
{
    uint32_t ulPreviousMask;

    // brPORTA = 0x01;
    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /* Pend a context switch. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
    }
    // brPORTA = 0x00;
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
}
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__(( weak )) void vPortSetupTimerInterrupt( void )
{
    /* Calculate the constants required to configure the tick interrupt. */
    #if( configUSE_TICKLESS_IDLE == 1 )
    {
        ulTimerCountsForOneTick = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR;
    }
    #endif /* configUSE_TICKLESS_IDLE */

    /* Stop and reset the SysTick. */
    portNVIC_SYSTICK_CTRL_REG = 0UL;
    portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

    /* Configure SysTick to interrupt at the requested rate. */
    portNVIC_SYSTICK_LOAD_REG = ( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL;
    portNVIC_SYSTICK_CTRL_REG = portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT;
}
/*-----------------------------------------------------------*/

#if( configUSE_TICKLESS_IDLE == 1 )

    __attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
    TickType_t xModifiableIdleTime;

        /* Make sure the SysTick reload value does not overflow the counter. */
        if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
        {
            xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
        }

        /* Stop the SysTick momentarily.  The time the SysTick is stopped for
        is accounted for as best it can be, but using the tickless mode will
        inevitably result in some tiny drift of the time maintained by the
        kernel with respect to calendar time. */
        portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

        /* Calculate the reload value required to wait xExpectedIdleTime
        tick periods.  -1 is used because this code will execute part way
        through one of the tick periods. */
        ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
        if( ulReloadValue > ulStoppedTimerCompensation )
        {
            ulReloadValue -= ulStoppedTimerCompensation;
        }

        /* Enter a critical section but don't use the taskENTER_CRITICAL()
        method as that will mask interrupts that should exit sleep mode. */
        //ARM M0: __asm volatile( "cpsid i" ::: "memory" );
        //ARM M0: __asm volatile( "dsb" );
        //ARM M0: __asm volatile( "isb" );
        
        __asm volatile ("psrclr ie" ::: "memory");
        __asm volatile ("sync" ::: "memory");

        /* If a context switch is pending or a task is waiting for the scheduler
        to be unsuspended then abandon the low power entry. */
        if( eTaskConfirmSleepModeStatus() == eAbortSleep )
        {
            /* Restart from whatever is left in the count register to complete
            this tick period. */
            portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

            /* Restart SysTick. */
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

            /* Reset the reload register to the value required for normal tick
            periods. */
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

            /* Re-enable interrupts - see comments above the cpsid instruction()
            above. */
            //ARM M0: __asm volatile( "cpsie i" ::: "memory" );
            __asm volatile("psrset ie" ::: "memory");
        }
        else
        {
            /* Set the new reload value. */
            portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

            /* Clear the SysTick count flag and set the count value back to
            zero. */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

            /* Restart SysTick. */
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

            /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
            set its parameter to 0 to indicate that its implementation contains
            its own wait for interrupt or wait for event instruction, and so wfi
            should not be executed again.  However, the original expected idle
            time variable must remain unmodified, so a copy is taken. */
            xModifiableIdleTime = xExpectedIdleTime;
            configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
            if( xModifiableIdleTime > 0 )
            {
                //ARM M0: __asm volatile( "dsb" ::: "memory" );
                //ARM M0: __asm volatile( "wfi" );
                //ARM M0: __asm volatile( "isb" );
                __asm volatile( "wait" );
            }
            configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

            /* Re-enable interrupts to allow the interrupt that brought the MCU
            out of sleep mode to execute immediately.  see comments above
            __disable_interrupt() call above. */
            //__asm volatile( "cpsie i" ::: "memory" );
            //__asm volatile( "dsb" );
            //__asm volatile( "isb" );
            __asm volatile("psrset ie" ::: "memory");
            __asm volatile("sync");

            /* Disable interrupts again because the clock is about to be stopped
            and interrupts that execute while the clock is stopped will increase
            any slippage between the time maintained by the RTOS and calendar
            time. */
            // ARM M0: __asm volatile( "cpsid i" ::: "memory" );
            // ARM M0: __asm volatile( "dsb" );
            // ARM M0: __asm volatile( "isb" );
            __asm volatile("psrclr ie" ::: "memory");

            /* Disable the SysTick clock without reading the
            portNVIC_SYSTICK_CTRL_REG register to ensure the
            portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
            the time the SysTick is stopped for is accounted for as best it can
            be, but using the tickless mode will inevitably result in some tiny
            drift of the time maintained by the kernel with respect to calendar
            time*/
            portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

            /* Determine if the SysTick clock has already counted to zero and
            been set back to the current reload value (the reload back being
            correct for the entire expected idle time) or if the SysTick is yet
            to count to zero (in which case an interrupt other than the SysTick
            must have brought the system out of sleep mode). */
            if( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
            {
                uint32_t ulCalculatedLoadValue;

                /* The tick interrupt is already pending, and the SysTick count
                reloaded with ulReloadValue.  Reset the
                portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
                period. */
                ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

                /* Don't allow a tiny value, or values that have somehow
                underflowed because the post sleep hook did something
                that took too long. */
                if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
                {
                    ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
                }

                portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

                /* As the pending tick will be processed as soon as this
                function exits, the tick value maintained by the tick is stepped
                forward by one less than the time spent waiting. */
                ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
            }
            else
            {
                /* Something other than the tick interrupt ended the sleep.
                Work out how long the sleep lasted rounded to complete tick
                periods (not the ulReload value which accounted for part
                ticks). */
                ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

                /* How many complete tick periods passed while the processor
                was waiting? */
                ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

                /* The reload value is set to whatever fraction of a single tick
                period remains. */
                portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
            }

            /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
            again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
            value. */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
            vTaskStepTick( ulCompleteTickPeriods );
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

            /* Exit with interrpts enabled. */
            // ARM M0: __asm volatile( "cpsie i" ::: "memory" );
            __asm volatile("psrset ie" ::: "memory");
        }
    }

#endif /* configUSE_TICKLESS_IDLE */
