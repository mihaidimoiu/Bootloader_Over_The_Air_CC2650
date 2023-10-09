/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include "driverlib/sys_ctrl.h"
#include "flash_rw.h"

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <driverlib/rf_prop_mailbox.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;
static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
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

enum rf_mode  {
    RX_MODE,
    TX_MODE,
};

volatile enum rf_mode mode = RX_MODE;

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             100 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


/***** Defines *****/
#define RX_TASK_STACK_SIZE 1024
#define RX_TASK_PRIORITY   1

#define UART_RX_TASK_STACK_SIZE 1024
#define UART_RX_TASK_PRIORITY   2

#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   3

#define UART_TX_TASK_STACK_SIZE 1024
#define UART_TX_TASK_PRIORITY   4

/* Valori ASCII*/
#define CHAR_LINEFEED                         0x0A   //CR
#define CHAR_LINE_END_1                       0x0D   // Enter
#define CHAR_LINE_END_2                       0x03   // Enter numpad
#define CHAR_SPACE                            0x20

//#define UART_TX_BUFFER_SIZE 256
//char uartTxBuffer[UART_TX_BUFFER_SIZE];

/* TX Configuration */
#define PAYLOAD_LENGTH      100
#define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */

typedef struct{
    uint8_t pktNo;
    uint8_t cmd;
    uint8_t address[4];
    uint8_t app_size[4];
    uint8_t dim_payload;
    uint8_t payload[80];
    uint8_t crc[4];
} Paket;

void Paket_init(Paket *p){
    p->pktNo = 0;
}

Paket pkt,pktRx;
/***** Prototypes *****/
static void txTaskFunction(UArg arg0, UArg arg1);
static void rxTaskFunction(UArg arg0, UArg arg1);
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void uartRxTaskFunction(UArg arg0, UArg arg1);
static void uartTxTaskFunction(UArg arg0, UArg arg1);
void writePayLoad(Paket*, uint8_t);
void saltProgram();
void resetarePlaca();
uint32_t crc32_for_byte(uint32_t);
void crc32(const void*, size_t, uint32_t*);

/***** Variable declarations *****/
static Task_Params rxTaskParams;
Task_Struct rxTask;    /* not static so you can see in ROV */
static uint8_t rxTaskStack[RX_TASK_STACK_SIZE];

static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static Task_Params uartRxTaskParams;
Task_Struct uartRxTask;    /* not static so you can see in ROV */
static uint8_t uartRxTaskStack[UART_RX_TASK_STACK_SIZE];

static Task_Params uartTxTaskParams;
Task_Struct uartTxTask;    /* not static so you can see in ROV */
static uint8_t uartTxTaskStack[UART_TX_TASK_STACK_SIZE];

Semaphore_Struct semTxStruct;
Semaphore_Handle semTxHandle;

Semaphore_Struct semRxStruct;
Semaphore_Handle semRxHandle;

Semaphore_Struct semUartRxStruct;
Semaphore_Handle semUartRxHandle;

static RF_Object rfObject;
static RF_Handle rfHandle;
static RF_CmdHandle rfRxCmd;
static RF_CmdHandle rfTxCmd;

UART_Handle uart = NULL;
UART_Params uartParams;

Error_Block eb;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
    #pragma DATA_ALIGN (rxDataEntryBuffer, 4);
        static uint8_t rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                                 MAX_LENGTH,
                                                                 NUM_APPENDED_BYTES)];
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
uint8_t packetReady = 0;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
uint32_t time;
static uint8_t txPacket[PAYLOAD_LENGTH];

static PIN_Handle pinHandle;

//static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

Semaphore_Struct semStruct;
Semaphore_Handle semHandle;

uint32_t flash_address = 0;
uint32_t app_size = 0;
uint32_t i = 0;
uint8_t flash_erased = 0;
uint8_t ack = 0x55;
uint8_t nck = 0x33;
uint32_t crc_calculat = 0;
uint32_t crc_primit = 0;
/* Function Defs */
void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId) {
    uint32_t currVal = 0;

    /* Debounce logic, only toggle if the button is still pushed (low) */
    CPUdelay(8000*50);
    if (!PIN_getInputValue(pinId)) {
        /* Toggle LED based on the button pressed */
        switch (pinId) {
            case Board_BUTTON0:
                RF_cancelCmd(rfHandle, rfRxCmd, 0);
                RF_close(rfHandle);
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

uint32_t crc32_for_byte(uint32_t r) {
    uint8_t j ;
    for(j = 0; j < 8; ++j)
        r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
    return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
    static uint32_t table[0x100];
    uint32_t i;
    if(!*table)
        for(i = 0; i < 0x100; ++i)
            table[i] = crc32_for_byte(i);
    for(i = 0; i < n_bytes; ++i)
        *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
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

void UartRxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartRxTaskParams);
    uartRxTaskParams.stackSize = UART_RX_TASK_STACK_SIZE;
    uartRxTaskParams.priority = UART_RX_TASK_PRIORITY;
    uartRxTaskParams.stack = &uartRxTaskStack;
    uartRxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartRxTask, uartRxTaskFunction, &uartRxTaskParams, &eb);
}

void UartTxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&uartTxTaskParams);
    uartTxTaskParams.stackSize = UART_TX_TASK_STACK_SIZE;
    uartTxTaskParams.priority = UART_TX_TASK_PRIORITY;
    uartTxTaskParams.stack = &uartTxTaskStack;
    uartTxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&uartTxTask, uartTxTaskFunction, &uartTxTaskParams, &eb);

}


void RxTask_init(PIN_Handle ledPinHandle) {
    pinHandle = ledPinHandle;

    Task_Params_init(&rxTaskParams);
    rxTaskParams.stackSize = RX_TASK_STACK_SIZE;
    rxTaskParams.priority = RX_TASK_PRIORITY;
    rxTaskParams.stack = &rxTaskStack;
    rxTaskParams.arg0 = (UInt)1000000;

    Task_construct(&rxTask, rxTaskFunction, &rxTaskParams, &eb);

}

void TxTask_init(PIN_Handle inPinHandle)
{
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, &eb);
}

void Uart_Init()
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
        while(1);
    }
}

static void rxTaskFunction(UArg arg0, UArg arg1)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    RF_cmdPropRx.pQueue = &dataQueue;           /* Set the Data Entity queue for received data */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;        /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    if (!rfHandle) {
        /* Request access to the radio */
        rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

        /* Set the frequency */
        RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
    }

    while (1) {
    /* Enter RX mode and stay forever in RX */
        rfRxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropRx, RF_PriorityNormal, &callback, IRQ_RX_ENTRY_DONE);

        Semaphore_pend(semRxHandle, BIOS_WAIT_FOREVER);     //Suspenda executia task-ului dupa ce s-a intrat in modul RX, va da o intrerupere hardware cand se va primi ceva
    }

}

static void txTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    while(1)
    {
        Semaphore_pend(semTxHandle, BIOS_WAIT_FOREVER);
        if (!rfHandle) {
            /* Request access to the radio */
            rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

            /* Set the frequency */
            RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);
        }

        /* Get current time */
        time = RF_getCurrentTime();

        /* Set absolute TX time to utilize automatic power management */
        time += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = time;

        /* Send packet */
        //RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        rfTxCmd = RF_postCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        RF_EventMask result = RF_pendCmd(rfHandle, rfTxCmd, (RF_EventCmdDone | RF_EventCmdError | RF_EventLastCmdDone |
                RF_EventCmdAborted | RF_EventCmdCancelled | RF_EventCmdStopped));

        if (!(result & RF_EventLastCmdDone))
        {
            /* Error */
            while(1);
        }

        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

        /* Stergerea bufferului */
        memset(txPacket, 0, sizeof(txPacket));

        Semaphore_post(semRxHandle);    //Deblocheaza task-ul RF de RX
    }
}

static void uartTxTaskFunction(UArg arg0, UArg arg1)
{
    unsigned char input;
    uint8_t charIndex = 0;
    uint8_t dim_paket;

    while (1) {
        UART_read(uart, &input, 1);
        pkt.pktNo++;
        pkt.cmd = input;
        switch(input){
        case 0x10:
            UART_read(uart, &pkt.address, 4);
            UART_read(uart, &pkt.app_size, 4);
            memcpy(txPacket, &pkt, sizeof(Paket));
            RF_cancelCmd(rfHandle, rfRxCmd, 0);
            Semaphore_post(semTxHandle);
            charIndex = 0;
            break;
        case 0x11:
            UART_read(uart, &pkt.dim_payload, 1);
            dim_paket = pkt.dim_payload;
            while(dim_paket--){
                UART_read(uart, &pkt.payload[charIndex++], 1);
            }
            UART_read(uart, &pkt.crc, 4);
            memcpy(txPacket, &pkt, sizeof(Paket));
            charIndex = 0;
            RF_cancelCmd(rfHandle, rfRxCmd, 0);
            Semaphore_post(semTxHandle);

            break;
        case 0x19:
            memcpy(txPacket, &pkt, sizeof(Paket));
            RF_cancelCmd(rfHandle, rfRxCmd, 0);
            Semaphore_post(semTxHandle);
            charIndex = 0;
            break;
        default:
            break;
        }
    }
}

static void uartRxTaskFunction(UArg arg0, UArg arg1)
{
    while (1) {
        Semaphore_pend(semUartRxHandle, BIOS_WAIT_FOREVER);     //Blocheaza resursele task-ului si verifica daca s-a primit ceva iar apoi scrie pe seriala si reseteaza flagul
        if (packetReady) {
            writePayLoad(&pktRx, packetLength);
            packetReady = 0;
        }
    }
}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(&pktRx, packetDataPointer, sizeof(pktRx));
        crc_primit |= (uint32_t)pktRx.crc[0] << 24;
        crc_primit |= (uint32_t)pktRx.crc[1] << 16;
        crc_primit |= (uint32_t)pktRx.crc[2] << 8;
        crc_primit |= (uint32_t)pktRx.crc[3] ;
        crc32(pktRx.crc, 4, &crc_calculat);
        if(crc_calculat == crc_primit)
            packetReady = 1;    //Scrierea flagului daca s-a primit un pachet
        else
            packetReady = 0;    //Scrierea flagului daca s-a primit un pachet

        RFQueue_nextEntry();

        Semaphore_post(semUartRxHandle);    //Deblocheaza task-ul pentru scrierea pe UART
    }
}

int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Error_init(&eb);
    Uart_Init();

    /* Open LED pins */
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

    Semaphore_Params semParams;  //Parametrii semafor

    Semaphore_Params_init(&semParams);  //Initializare parametrii
    Semaphore_construct(&semTxStruct, 0, &semParams);   //Creare structura semafor
    Semaphore_construct(&semRxStruct, 0, &semParams);
    Semaphore_construct(&semUartRxStruct, 0, &semParams);

    semTxHandle = Semaphore_handle(&semTxStruct);   //Convertirea structurii in handle
    semRxHandle = Semaphore_handle(&semRxStruct);
    semUartRxHandle = Semaphore_handle(&semUartRxStruct);

    /* Verificare daca s-a creat obiectul pt semafoare */
    if(semTxHandle == NULL){
        while(1);
    }

    if(semRxHandle == NULL){
        while(1);
    }

    if(semUartRxHandle == NULL){
        while(1);
    }

    /* Initializare task-uri */
    RxTask_init(ledPinHandle);

    TxTask_init(ledPinHandle);

    UartRxTask_init(ledPinHandle);

    UartTxTask_init(ledPinHandle);

    Paket_init(&pkt);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void writePayLoad(Paket *packet, uint8_t length) {
    uint8_t cmd = packet->cmd;
    uint8_t dim_paket = 0;
    uint8_t count = 0;
    uint8_t led = 0;
    uint8_t currVal;
//    uint8_t raspuns = 0;
    switch(cmd){
    case 0x19:
        PIN_setOutputValue(ledPinHandle, Board_LED0, 0);
        PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
        for (led = 0 ; led < 15 ; led++){
            currVal =  PIN_getOutputValue(Board_LED0);
            PIN_setOutputValue(ledPinHandle, Board_LED0, !currVal);
            currVal =  PIN_getOutputValue(Board_LED1);
            PIN_setOutputValue(ledPinHandle, Board_LED1, !currVal);
            CPUdelay(1000000);
        }
        PIN_setOutputValue(ledPinHandle, Board_LED0, 0);
        PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
        break;
    case 0x10:
        flash_address |= (uint32_t)packet->address[0] << 24;
        flash_address |= (uint32_t)packet->address[1] << 16;
        flash_address |= (uint32_t)packet->address[2] << 8;
        flash_address |= (uint32_t)packet->address[3] ;

        app_size |= (uint32_t)packet->app_size[0] << 24;
        app_size |= (uint32_t)packet->app_size[1] << 16;
        app_size |= (uint32_t)packet->app_size[2] << 8;
        app_size |= (uint32_t)packet->app_size[3];
        RF_cancelCmd(rfHandle, rfRxCmd, 0);
        pkt.cmd = 0x09;
        memcpy(txPacket, &pkt, sizeof(Paket));
        Semaphore_post(semTxHandle);
        break;
    case 0x11:
        dim_paket = packet->dim_payload;
        while(dim_paket--){
            if(i == 4096 * flash_erased){
                if(eraseFlashSector(flash_address + (4096 * flash_erased)) < 0){
                    while(1)
                        PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
                }
            }
            if(flashReadyFSM()){
                if(writeFlash(flash_address + i, &packet->payload[count], 1, false) < 0){
                    while(1)
                        PIN_setOutputValue(ledPinHandle, Board_LED1, 1);
                }
            }
            count += 1;
            i += 1;
        }
        PIN_setOutputValue(ledPinHandle, Board_LED0,!PIN_getOutputValue(Board_LED0));
        RF_cancelCmd(rfHandle, rfRxCmd, 0);
        pkt.cmd = 0x09;
        memcpy(txPacket, &pkt, sizeof(Paket));
        Semaphore_post(semTxHandle);
        break;
    default:
        break;
    }
}
