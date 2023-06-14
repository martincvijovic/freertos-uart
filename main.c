/**
 * @file    main.c
 * @author  Martin Cvijovic (cm170558d@student.etf.bg.ac.rs)
 * @date    2021
 * @brief   SRV Projekat
 *
 * Every 1500ms the ADC channels A0 and A1 conversion is being triggered. The last four conversion results from both channels
 * are used to calculate the mean channel conversion value. For each channel, if the currently read
 * value is greater than the average calculated value, the corresponding LED is turned on. The threshold value can be directly
 * provided via UART or incremented/decremented using S1 and S2 buttons.
 *
 * @version [1.0 @ 07/2021] Initial version
 */

/* Standard includes. */
#include <ETF5529_HAL/hal_ETF_5529.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware includes. */
#include "msp430.h"

/* Task priorities */
#define mainTASK_1_PRIORITY     (2) // The second highest priority, this task is invoked after successful ADC conversion to process the results
#define mainTASK_3_PRIORITY     (3) // Highest priority, the button debouncing task
#define mainTASK_2_PRIORITY     (1) // Lowest priority, this task parses the user input for the new threshold value
#define mainTASK_TIMER_PRIORITY (1) // Also lowest priority, the ADC conversion starter task

/* UART message constants */
#define mainUART_MESSAGE_LED3_ON  ("Ukljucena LED3")
#define mainUART_MESSAGE_LED3_OFF ("Iskljucena LED3")
#define mainUART_MESSAGE_LED4_ON  ("Ukljucena LED4")
#define mainUART_MESSAGE_LED4_OFF ("Iskljucena LED4")

#define mainADC_SAMPLE_RATE_MS  (1500) // ADC conversion sample rate

#define mainQUEUE_LENGTH        (20)   // Queue lengths (QueueT1 and QueueT2)

// **************************** CUSTOM DATA TYPES ************************************ //

/**
 * @brief LED states
 * Required to notify the PC via UART about the current states of the diodes.
 */
enum LED_STATE { OFF, ON };
enum LED_STATE led3State, led4State;

/**
 * @brief Data received from ADC conversion
 *
 * Contains information about which channel performed the conversion and the conversion result.
 */
enum ADC_CHANNEL { ADC_CHANNEL_0, ADC_CHANNEL_1 };
typedef struct
{
    enum ADC_CHANNEL   channel;
    uint16_t           data;
} adcDataBuffer;

/*
 * @brief Data used by Task 3 to notify the Task 2 about updating the LED threshold value.
 * The value can either be incremented/decremented or specified directly via UART;
 */
enum THRESHOLD_UPDATE_TYPE { INCREMENT, DECREMENT, SPECIFY_DIRECT };
typedef struct
{
    enum THRESHOLD_UPDATE_TYPE      thresholdUpdateType;
    uint16_t                        directSpecifiedValue;
} userInteractionData;

/* Global variables */
TaskHandle_t        xTask1TaskHandle;         // Used by ADC ISR to notify Task 1 when a conversion is finished
QueueHandle_t       xQueueT1;                 // ADC conversion result queue
QueueHandle_t       xQueueT2;                 // New LED threshold queue
QueueHandle_t       xQueueMailboxChannel0;    // Mailbox for notifying Task 1 from ADC ISR
QueueHandle_t       xQueueMailboxChannel1;    // _________||__________

xSemaphoreHandle    xFinishedADCConversionSemaphore;  // Used to release Task 1 to process the ADC conversion data
xSemaphoreHandle    xButtonEventSemaphore;            // Used to signal successful button press

static uint16_t     conversionCounts[2] = {0};        // Successful conversions Count, conversionCounts[i] denotes channel i.
volatile uint16_t   meanConversionValues[2] = {0};    // Mean conversion value
volatile uint16_t   conversionValues[2][4] = {0};     // 4 last conversion values
volatile uint16_t   ledThresholdValue = 2048;         // LED toggle threshold value (medium initially)


/* Function declarations */
static void prvSetupHardware(void);

// **************************** TASK FUNCTIONS ************************************ //

static void notifyDiodeStates()
{
    char* msg3 = (led3State == ON) ? mainUART_MESSAGE_LED3_ON : mainUART_MESSAGE_LED3_OFF;
    char* msg4 = (led4State == ON) ? mainUART_MESSAGE_LED4_ON : mainUART_MESSAGE_LED4_OFF;

    // Sending messages
    while(*msg3 != 0) // while the char pointer is not 0
    {
        while (!(UCA1IFG & UCTXIFG)); // while not UART free to send

        UCA1TXBUF = *msg3;
        msg3 += 1;
    }
    while(*msg4 != 0) // while the char pointer is not 0
    {
        while (!(UCA1IFG & UCTXIFG)); // while not UART free to send

        UCA1TXBUF = *msg4;
        msg4 += 1;
    }
}

/*
 * @brief Triggered by AD conversion notification. Calculates the mean value of the last 4 conversion results for
 * both channels. Outputs the mean values into mailboxes.
 */
static void prvTask1Function(void *pvParameters)
{
    adcDataBuffer receivedAdcData;

    for ( ;; )
    {
        xSemaphoreTake(xFinishedADCConversionSemaphore, portMAX_DELAY); // awaits for successful ADC conversion

        // Examining the queue (may contain 1 or 2 messages)
        while (xQueueReceive(xQueueT1, &receivedAdcData, 0) == pdTRUE)
        {
            int channel = (receivedAdcData.channel == ADC_CHANNEL_0) ? 0 : 1;
            conversionValues[channel][conversionCounts[channel]++] = receivedAdcData.data;

            uint16_t mean = 0;
            int i;
            for (i = 0; i < 3; i++)
            {
                mean += conversionValues[channel][i];
            }
            mean /= 4;
            meanConversionValues[channel] = mean;

            if (conversionCounts[channel] > 3)
            {
                conversionCounts[channel] = 0;
            }

            if (channel == 0)
            {
                xQueueOverwrite(xQueueMailboxChannel0, &mean);
            }
            else
            {
                xQueueOverwrite(xQueueMailboxChannel1, &mean);
            }
        }
    }
}

/*
 * @brief Handles the change of the LED threshold value. Triggered by the information from userInteractionData message received
 * via xQueueT2.
 * Also handles the diode state.
 */
static void prvTask2Function(void *pvParameters)
{
    userInteractionData userInteractionReceivedMessage;
    int meanConversionValue;

    for (;;)
    {

        //receiving mean conversion values
        int mean;
        if (xQueueReceive(xQueueMailboxChannel0, &mean, 0) == pdTRUE)
        {
            if (mean > ledThresholdValue)
            {
                halSET_LED(LED4);
                led4State = ON;
            }
            else
            {
                halCLR_LED(LED4);
                led4State = OFF;
            }

        }
        if (xQueueReceive(xQueueMailboxChannel1, &mean, 0) == pdTRUE)
        {
            if (mean > ledThresholdValue)
            {
                halSET_LED(LED3);
                led3State = ON;
            }
            else
            {
                halCLR_LED(LED3);
                led3State = OFF;
            }
        }

        if (xQueueReceive(xQueueT2, &userInteractionReceivedMessage, 0) == pdTRUE)
        {
            /* Handling threshold value */
            if (userInteractionReceivedMessage.thresholdUpdateType == SPECIFY_DIRECT)
            {
                ledThresholdValue = userInteractionReceivedMessage.directSpecifiedValue * 16;
            }
            else if (userInteractionReceivedMessage.thresholdUpdateType == INCREMENT)
            {
                ledThresholdValue += 50;

                if (ledThresholdValue > 4095)
                {
                    ledThresholdValue = 4095;
                }

                notifyDiodeStates();
            }
            else // decrement
            {
                ledThresholdValue -= 50;

                if (ledThresholdValue < 0)
                {
                    ledThresholdValue = 0;
                }

                notifyDiodeStates();
            }

        }
    }
}

/*
 * @brief Handles buttons S1 and S2. Notifies the Task 2 via xQueueT2 if and which button is pressed.
 */
static void prvTask3Function(void *pvParameters)
{
    for ( ;; )
    {
        xSemaphoreTake(xButtonEventSemaphore, portMAX_DELAY);

        int i;
        for (i = 0; i < 1000; i++); // debouncing

        bool isStillButtonPressed = false;

        userInteractionData data;
        data.directSpecifiedValue = 0;

        if (((P1IN & 0x02) >> 1) == 0)  // SW2 keypress
        {
            data.thresholdUpdateType = INCREMENT;
            isStillButtonPressed = true;
        }

        if (((P2IN & 0x02) >> 1) == 0)  // SW1 keypress
        {
            data.thresholdUpdateType = DECREMENT;
            isStillButtonPressed = true;
        }

        if (isStillButtonPressed)
        {
            xQueueSendToBack(xQueueT2, &data, portMAX_DELAY);
        }
    }
}

/*
 * @brief Triggers the A/D conversion every 1500ms on channels A0 and A1.
 */
static void prvTaskTimerFunction(void *pvParameters)
{
    for (;;)
    {
        // Trigger the A/D conversion on both channels
        ADC12CTL0 &= ~ADC12SC;
        ADC12CTL0 |= ADC12SC;
        // Sleep for 1500ms
        vTaskDelay(mainADC_SAMPLE_RATE_MS / portTICK_PERIOD_MS);
    }
}

/*
 * @brief The main function. Performs initial hardware initialization and task initialization.
 */
void main(void)
{
    //System_printf("test");
    /* Initial hardware setup */
    prvSetupHardware();

    /* Tasks initialization */
    if(xTaskCreate(prvTask1Function,
                 "Task 1",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK_1_PRIORITY,
                 NULL
               ) != pdPASS) while(1);

    if(xTaskCreate(prvTask2Function,
                 "Task 2",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK_2_PRIORITY,
                 NULL
               ) != pdPASS) while(1);

    if(xTaskCreate(prvTask3Function,
                 "Task 3",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK_3_PRIORITY,
                 NULL
               ) != pdPASS) while(1);

    if(xTaskCreate(prvTaskTimerFunction,
                 "Task Timer",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 mainTASK_TIMER_PRIORITY,
                 NULL
               ) != pdPASS) while(1);

    /* Creating global variables */
    xQueueT1 = xQueueCreate(mainQUEUE_LENGTH, sizeof(adcDataBuffer));
    xQueueT2 = xQueueCreate(mainQUEUE_LENGTH, sizeof(userInteractionData));
    xQueueMailboxChannel0 = xQueueCreate(1, sizeof(uint16_t));
    xQueueMailboxChannel1 = xQueueCreate(1, sizeof(uint16_t));
    xButtonEventSemaphore = xSemaphoreCreateBinary();
    xFinishedADCConversionSemaphore = xSemaphoreCreateBinary();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well then this line will never be reached.  If it is reached
    then it is likely that there was insufficient (FreeRTOS) heap memory space
    to create the idle task.  This may have been trapped by the malloc() failed
    hook function, if one is configured. */	
    for( ;; );

    /*
     * NOTE: Trapping the main in a while(1) loop above is a bad practice as it's inconvenient
     * to hang the process on errors. TODO: consider some error reporting/signaling methods such as blinking a LED.
     */
}

/**
 * @brief Initial hardware configuration. Configures S1, S2, ADC, UART and LEDs.
 */
static void prvSetupHardware( void )
{
    taskDISABLE_INTERRUPTS();

    /* Disabling watchdog timer */
    WDTCTL = WDTPW + WDTHOLD;

    hal430SetSystemClock( configCPU_CLOCK_HZ, configLFXT_CLOCK_HZ );

    /* S1 init */
    P2DIR &= ~0x02;                 // Set direction to input
    P2REN |= 0x02;                  // Enable pull up
    P2OUT |= 0x02;
    P2IE  |= 0x02;                  // Interrupt enable
    P2IFG &=~0x02;
    P2IES |= 0x02;                  // Falling edge interrupt

    /* S2 init */
    P1DIR &= ~0x02;                 // Set direction to input
    P1REN |= 0x02;                  // Enable pull up
    P1OUT |= 0x02;
    P1IFG &=~0x02;                  // Interrupt enable
    P1IE  |= 0x02;
    P1IES |= 0x02;                  // Falling edge interrupt

    /* ADC init */
    ADC12CTL0 = ADC12SHT03 + ADC12ON + ADC12MSC ;       // Sampling time, ADC12 on
    ADC12CTL1 = ADC12CONSEQ_1 + ADC12SHP;
                                                 //  Sampling sequence of channels
    ADC12IE = 0x02;                       // Enable interrupt for both channels
    ADC12MCTL0 |= ADC12INCH_0;               // ADCMEM0 store result from xChannel 0
    ADC12MCTL1 |= ADC12INCH_1;               // ADCMEM1 store result from xChannel 1
    ADC12MCTL1 |= ADC12EOS;                  // Channel 1 is last xChannel in sequence
    ADC12CTL0 |= ADC12ENC;
    P6SEL |= 0x03;                      // P6.0 and P6.1 ADC option select

    /* Initialize UART */
    P4SEL |= BIT4+BIT5;                    // P4.4,5 = USCI_AA TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BRW = 1041;                         // 1MHz - Baudrate 9600
    UCA1MCTL |= UCBRS_6 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                       // Enable USCI_A1 RX interrupt

    /* LED init */
    vHALInitLED();

    taskENABLE_INTERRUPTS();
}

/*
 * @brief PORT1 ISR, triggers on key press
 */
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1ISR (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if((P1IFG & 0x02) == 0x02) {
        xSemaphoreGiveFromISR(xButtonEventSemaphore, &xHigherPriorityTaskWoken);
    }

    P1IFG &= ~0x02; // clear IFG flag

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // trigger scheduler if higher priority task is woken
}

/*
 * @brief PORT2 ISR, triggers on key press
 */
void __attribute__ ((interrupt(PORT2_VECTOR))) PORT2ISR (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if((P2IFG & 0x02) == 0x02) {
        xSemaphoreGiveFromISR(xButtonEventSemaphore, &xHigherPriorityTaskWoken);
    }

    P2IFG &= ~0x02; // clear IFG flag

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken ); // trigger scheduler if higher priority task is woken
}

/*
 * @brief UART ISR, triggers on received message
 */
void __attribute__ ((interrupt(USCI_A1_VECTOR))) UARTISR (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (UCA1IV == USCI_UCRXIFG) // if received messagee
    {
        userInteractionData data;
        data.directSpecifiedValue = UCA1RXBUF;
        data.thresholdUpdateType = SPECIFY_DIRECT;

        xQueueSendToBackFromISR(xQueueT2, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*
 * @brief ADC ISR. Used when a successful conversion occurs. The result is contained in a adcDataBuffer type and
 * the Task 1 is notified.
 */
void __attribute__ ((interrupt( ADC12_VECTOR ))) ADC12ISR (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    adcDataBuffer result;

    switch(__even_in_range(ADC12IV,34))
    {
        case  0: break;                           // Vector  0:  No interrupt
        case  2: break;                           // Vector  2:  ADC overflow
        case  4: break;                           // Vector  4:  ADC timing overflow
        case  6:                                  // Vector  6:  ADC12IFG0
            break;
        case  8:                                  // Vector  8:  ADC12IFG1
            // Channel 0
            result.channel = ADC_CHANNEL_0;
            result.data = ADC12MEM0;
            xQueueSendToBackFromISR(xQueueT1, &result, &xHigherPriorityTaskWoken);

            // Channel 1
            result.channel    = ADC_CHANNEL_1;
            result.data       = ADC12MEM1;
            xQueueSendToBackFromISR(xQueueT1, &result, &xHigherPriorityTaskWoken);

            // Releasing semaphore for task 1
            xSemaphoreGive(xFinishedADCConversionSemaphore);
            break;
        case 10: break;                           // Vector 10:  ADC12IFG2
        case 12: break;                           // Vector 12:  ADC12IFG3
        case 14: break;                           // Vector 14:  ADC12IFG4
        case 16: break;                           // Vector 16:  ADC12IFG5
        case 18: break;                           // Vector 18:  ADC12IFG6
        case 20: break;                           // Vector 20:  ADC12IFG7
        case 22: break;                           // Vector 22:  ADC12IFG8
        case 24: break;                           // Vector 24:  ADC12IFG9
        case 26: break;                           // Vector 26:  ADC12IFG10
        case 28: break;                           // Vector 28:  ADC12IFG11
        case 30: break;                           // Vector 30:  ADC12IFG12
        case 32: break;                           // Vector 32:  ADC12IFG13
        case 34: break;                           // Vector 34:  ADC12IFG14
        default: break;
    }

    /* trigger scheduler if higher priority task is woken */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

