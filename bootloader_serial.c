/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
/* Example/Board Header files */
#include "Board.h"
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include<ti/drivers/UART.h>
#include "driverlib/sys_ctrl.h"
#define TASK_STACK_SIZE 512


/*
 * #include <stdarg.h>
#include <stdlib.h>
#include <string.h>
char* concat(int count, ...)
{
    va_list ap;
    int i;

    // Find required length to store merged string
    int len = 1; // room for NULL
    va_start(ap, count);
    for(i=0 ; i<count ; i++)
        len += strlen(va_arg(ap, char*));
    va_end(ap);

    // Allocate memory to concat strings
    char *merged = calloc(sizeof(char),len);
    int null_pos = 0;

    // Actually concatenate strings
    va_start(ap, count);
    for(i=0 ; i<count ; i++)
    {
        char *s = va_arg(ap, char*);
        strcpy(merged+null_pos, s);
        null_pos += strlen(s);
    }
    va_end(ap);

    return merged;
}
 */

Task_Struct taskStruct;
Char taskStack[TASK_STACK_SIZE];

UART_Handle uart;
UART_Params uartParams;

Error_Block eb;
Semaphore_Handle sem;

static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;


//#define flash_address 0x10000
#include "flash_rw.h"
void uart_init();

void fxn();

void saltProgram();
void resetarePlaca();

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config buttonPinTable[] = {
    Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

void uart_init()
{
    UART_Params_init(&uartParams);
    uartParams.baudRate = 9600;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.dataLength = UART_LEN_8;
    uartParams.parityType = UART_PAR_NONE;
    uartParams.stopBits = UART_STOP_ONE;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    //uartParams.readTimeout = 100000;
    uart = UART_open(0,&uartParams);

    if (uart == NULL)
    {
        System_abort("Error opening the UART");
    }
}

void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    uint32_t currVal = 0;

    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
        /* Toggle LED based on the button pressed */
        switch (pinId) {
            case Board_BUTTON0:
                UART_close(uart);
                saltProgram();
                break;

            case Board_BUTTON1:
                currVal =  PIN_getOutputValue(Board_LED1);
                PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);
                break;

            default:
                /* Do nothing */
                break;
        }
    }
}
uint32_t flash_address = 0;
uint32_t app_size = 0;
uint32_t i = 0;
uint32_t flash_erased = 0;
void fxn()
{
    unsigned char input;
    uint8_t currVal = 0;
    uint8_t flash_addr[4];
    uint8_t app_vector_size[4];
    uint8_t dim_paket;

    while (1)
    {
        UART_read(uart,&input,1);

        switch(input){
        case 0x19:
            UART_read(uart, &flash_addr, 4);
            UART_read(uart, &app_vector_size, 4);

            flash_address |= (uint32_t)flash_addr[0] << 24;
            flash_address |= (uint32_t)flash_addr[1] << 16;
            flash_address |= (uint32_t)flash_addr[2] << 8;
            flash_address |= (uint32_t)flash_addr[3];

            app_size |= (uint32_t)app_vector_size[0] << 24;
            app_size |= (uint32_t)app_vector_size[1] << 16;
            app_size |= (uint32_t)app_vector_size[2] << 8;
            app_size |= (uint32_t)app_vector_size[3];
            break;
        case 0x29:
            UART_read(uart, &dim_paket, 1);
            while(dim_paket--){
                UART_read(uart, &input, 1);

                if(i == 4096 * flash_erased){
//                    System_printf("erased flash pages %d\n", flash_erased);
//                    currVal =  PIN_getOutputValue(Board_LED0);
//                    PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
//                    System_printf("flash address %d\n", flash_address);
//                    System_printf("address to be erased %d\n", flash_address + (4096 * flash_erased));
//                    System_flush();
//                    currVal =  PIN_getOutputValue(Board_LED0);
//                    PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
//                    currVal =  PIN_getOutputValue(Board_LED1);
//                    PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);

                    if(eraseFlashSector(flash_address + (4096 * flash_erased)) < 0  ){
//                        System_printf("error erasing flash page %d\n", flash_erased);
//                        System_printf("address to be erased %d\n", flash_address + (4096 * flash_erased));
//                        System_flush();
//                        currVal =  PIN_getOutputValue(Board_LED0);
//                        PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
//                        currVal =  PIN_getOutputValue(Board_LED1);
//                        PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);
                    }
                    flash_erased += 1;

                }
                if( flashReadyFSM() ){
                    //System_printf("writing flash\n");
                    //System_flush();
                    if(writeFlash(flash_address + i, &input, sizeof(input), false) < 0){
                        //System_printf("error writing flash\n");
                        //System_flush();
                    }
                }
                i += 1;
            }
            currVal =  PIN_getOutputValue(Board_LED1);
            PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);

            break;

        case 0x39:
            UART_read(uart, &flash_addr, 4);
            flash_address |= (uint32_t)flash_addr[0] << 24;
            flash_address |= (uint32_t)flash_addr[1] << 16;
            flash_address |= (uint32_t)flash_addr[2] << 8;
            flash_address |= (uint32_t)flash_addr[3];
            eraseFlashSector(flash_address);
            break;
        case 0x58:
            UART_write(uart, "reset", 5);
            resetarePlaca();
            break;
        case 0x59:
           UART_close(uart);
           saltProgram();
           break;
        default:
            break;
        }
    }
}

void resetarePlaca(){
    SysCtrlSystemReset();
}

void saltProgram()
{
    // Change the below constant to match the application image intvec start.
    //             vvvvvv
    asm(" MOV R0, #0x10000 ");     // The .resetVecs or .intvecs for the app are
                                  // are placed at the constant #0xXXXX address
    asm(" LDR SP, [R0, #0x0] ");  // Load the initial stack pointer
    asm(" LDR R1, [R0, #0x4] ");  // Load the Reset vector address
    asm(" BX R1 ");               // Jump to the application Reset vector
}


int main()
{
    /* Call board init functions */
    Board_initGeneral();

    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if(!ledPinHandle) {
        System_abort("Error initializing board LED pins\n");
    }

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle) {
        System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0) {
        System_abort("Error registering button callback function");
    }

    uart_init();
    Error_init(&eb);

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASK_STACK_SIZE;
    taskParams.stack = &taskStack;
    taskParams.priority = 1;
    taskParams.arg0 = 1;
    Task_construct(&taskStruct, (Task_FuncPtr)fxn, &taskParams, &eb);
    BIOS_start();
    return(0);
}
