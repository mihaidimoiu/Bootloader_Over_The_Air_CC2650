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
#include "driverlib/flash.h"
#include "driverlib/vims.h"
#define TASK_STACK_SIZE 512

Task_Struct taskStruct;
Char taskStack[TASK_STACK_SIZE];

//Task_Struct taskStruct2;
//Char taskStack2[TASK_STACK_SIZE];

//Task_Struct taskStruct3;
//Char taskStack3[TASK_STACK_SIZE];

UART_Handle uart;
UART_Params uartParams;

//Task_Handle tsk0, tsk1;
Error_Block eb;
Semaphore_Handle sem;
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;


#define flash_address 0x10000

void uart_init();
void readFlashStaticData(uint32_t, uint8_t*, uint8_t);
int writeFlash(uint32_t, uint8_t*, uint8_t, bool);
int eraseFlashSector(uint32_t);
void readFlashNonCache(uint32_t, uint8_t*, uint8_t);
bool flashReadyFSM();
bool flashWriteProtect(uint32_t);
void fxn();

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
                resetarePlaca();
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


void fxn()
{
    unsigned char input;
    uint32_t i = 0;
    uint8_t flash_erased = 0;
    uint8_t currVal = 0;


    eraseFlashSector(flash_address);
    while (1)
    {
        UART_read(uart,&input,1);


        if(i == 4096 * flash_erased){
            flash_erased += 1;
            eraseFlashSector(flash_address + (4096 * flash_erased));
        }
        if( flashReadyFSM() ){
            //System_printf("writing flash\n");
            //System_flush();

            if(writeFlash(flash_address + i, &input, sizeof(input), false) < 0){
                currVal =  PIN_getOutputValue(Board_LED0);
                PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
                //System_printf("error writing flash\n");
                //System_flush();
            }
        }

       i++;
        /*if( flashReadyFSM() ){
            //System_printf("reading flash\n");
            //System_flush();
            readFlashNonCache(flash_address + i, &output, sizeof(uint8_t));
            //System_printf("%d %d\n", output, flash_address + i);
            //System_flush();
            UART_write(uart,&output,sizeof(output));
            //UART_write(uart,(void*)flash_address,sizeof(flash_address));
        }*/
    }
}

void resetarePlaca()
{
    // Change the below constant to match the application image intvec start.
    //             vvvvvv
    asm(" MOV R0, #0x10000 ");     // The .resetVecs or .intvecs for the app are
                                  // are placed at the constant #0xXXXX address
    asm(" LDR SP, [R0, #0x0] ");  // Load the initial stack pointer
    asm(" LDR R1, [R0, #0x4] ");  // Load the Reset vector address
    asm(" BX R1 ");               // Jump to the application Reset vector
}

bool flashReadyFSM(){
    if( FlashCheckFsmForReady() == FAPI_STATUS_FSM_READY){
        return true;
    }
    return false;
}

bool flashWriteProtect(uint32_t address){
    if( FlashProtectionGet(address) == FLASH_WRITE_PROTECT ){
        return false;
    }
    return true;
}

//-1 writeprotect
//-2 erase error
//-3 flash program error
int writeFlash(uint32_t address, uint8_t* buffer, uint8_t len, bool erase){
    if( !flashWriteProtect(address)){
        return -1;
    }

    if(erase == true)
        if(eraseFlashSector(address) < 0){
            return -2;
        }
    CPUcpsid();
    if(FlashProgram(buffer, address, len) != FAPI_STATUS_SUCCESS){
        return -3;
    }
    CPUcpsie();
    return 0;
}
//-2 error erase flash sector
int eraseFlashSector(uint32_t address){
    CPUcpsid();
    int status = FlashSectorErase(address);
    if(status == FAPI_STATUS_FSM_ERROR){
        CPUcpsie();
        return -1;
    }
    if(status == FAPI_STATUS_INCORRECT_DATABUFFER_LENGTH){
        CPUcpsie();
        return -2;
    }
    if(status == FAPI_STATUS_SUCCESS)
        CPUcpsie();
    return 0;
}
//Citire date scrise dinamic din flash
void readFlashNonCache(uint32_t adresa, uint8_t *buffer_citire, uint8_t lungime_buffer)
{
    VIMSModeSet(VIMS_BASE, VIMS_MODE_DISABLED);
    while(VIMSModeGet(VIMS_BASE) != VIMS_MODE_DISABLED);
    uint8_t *citire = (uint8_t *) adresa;
    while(lungime_buffer--)
       *(buffer_citire++)=*(citire++);
    VIMSModeSet(VIMS_BASE, VIMS_MODE_ENABLED);
}
//Citire date statice din flash
void readFlashStaticData(uint32_t adresa, uint8_t *buffer_citire, uint8_t lungime_buffer)
{
    uint8_t *citire = (uint8_t *) adresa;
    while(lungime_buffer--)
       *(buffer_citire++)=*(citire++);
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
