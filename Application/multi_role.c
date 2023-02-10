/******************************************************************************

 @file  multi_role.c

 @brief This file contains the multi_role sample application for use
 with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>

#if (!(defined FREERTOS) && !(defined __TI_COMPILER_VERSION__)) && !(defined(__clang__))
#include <intrinsics.h>
#endif

#include <ti/drivers/GPIO.h>
//#include <ti/drivers/utils/List.h>
#include <ti/drivers/NVS.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART2.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <Juxta2Profiles/juxta2_gatt_profile.h>

#include <ti_drivers_config.h>

#include "ti_ble_config.h"
#include "multi_role.h"
#include "lsm303agr_reg.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Application events
#define MR_EVT_CHAR_CHANGE         1
#define MR_EVT_KEY_CHANGE          2
#define MR_EVT_ADV_REPORT          3
#define MR_EVT_SCAN_ENABLED        4
#define MR_EVT_SCAN_DISABLED       5
#define MR_EVT_SVC_DISC            6
#define MR_EVT_ADV                 7
#define MR_EVT_PAIRING_STATE       8
#define MR_EVT_PASSCODE_NEEDED     9
#define MR_EVT_SEND_PARAM_UPDATE   10
#define MR_EVT_PERIODIC            11
#define MR_EVT_READ_RPA            12
#define MR_EVT_INSUFFICIENT_MEM    13
#define JUXTA_EVT_LED_TIMEOUT      14
#define JUXTA_TIME_UPDATED         15

#define JUXTA_LED_TIMEOUT_PERIOD        1 // ms
#define TIME_SERVICE_UUID               0xEFFE // see iOS > BLEPeripheralApp
#define JUXTA_CONFIG_SIZE               1 // uint32_t
#define JUXTA_CONFIG_OFFSET_LOGCOUNT    0
#define LSM303AGR_BOOT_TIME             5 // ms
#define SPI_HALF_PERIOD                 1 // us, Fs = 500kHz

// Internal Events for RTOS application
#define MR_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define MR_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

#define MR_ALL_EVENTS                        (MR_ICALL_EVT           | \
                                              MR_QUEUE_EVT)

// Supervision timeout conversion rate to miliseconds
#define CONN_TIMEOUT_MS_CONVERSION            10

// Task configuration
#define MR_TASK_PRIORITY                     1
#ifndef MR_TASK_STACK_SIZE
#define MR_TASK_STACK_SIZE                   1024
#endif

// Discovery states
typedef enum
{
    BLE_DISC_STATE_IDLE,                // Idle
    BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
    BLE_DISC_STATE_SVC,                 // Service discovery
    BLE_DISC_STATE_CHAR                 // Characteristic discovery
} discState_t;

// address string length is an ascii character for each digit +
#define MR_ADDR_STR_SIZE     15

// How often to perform periodic event (in msec)
#define MR_PERIODIC_EVT_PERIOD               1000

#define CONNINDEX_INVALID  0xFF

// Spin if the expression is not true
#define MULTIROLE_ASSERT(expr) if (!(expr)) multi_role_spin();

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    uint8_t event;    // event type
    void *pData;   // event data pointer
} mrEvt_t;

// Container to store paring state info when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t state;
    uint16_t connHandle;
    uint8_t status;
} mrPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
    uint8_t deviceAddr[B_ADDR_LEN];
    uint16_t connHandle;
    uint8_t uiInputs;
    uint8_t uiOutputs;
    uint32_t numComparison;
} mrPasscodeData_t;

// Scanned device information record
typedef struct
{
    uint8_t addrType;         // Peer Device's Address Type
    uint8_t addr[B_ADDR_LEN]; // Peer Device Address
} scanRec_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
    uint8_t event;
    uint8_t data[];
} mrClockEventData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
    uint32_t event;
    void *pBuf;
} mrGapAdvEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
    List_Elem elem;
    uint16_t connHandle;
} mrConnHandleEntry_t;

// Connected device information
typedef struct
{
    uint16_t connHandle;           // Connection Handle
    mrClockEventData_t *pParamUpdateEventData; // pointer to paramUpdateEventData
    uint16_t charHandle;           // Characteristic Handle
    uint8_t addr[B_ADDR_LEN];     // Peer Device Address
    Clock_Struct *pUpdateClock;         // pointer to clock struct
    uint8_t discState;            // Per connection deiscovery state
} mrConnRec_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

#define APP_EVT_EVENT_MAX  0x13
char *appEventStrings[] = { "APP_ZERO             ", "APP_CHAR_CHANGE      ",
                            "APP_KEY_CHANGE       ", "APP_ADV_REPORT       ",
                            "APP_SCAN_ENABLED     ", "APP_SCAN_DISABLED    ",
                            "APP_SVC_DISC         ", "APP_ADV              ",
                            "APP_PAIRING_STATE    ", "APP_SEND_PARAM_UPDATE",
                            "APP_PERIODIC         ", "APP_READ_RPA         ",
                            "APP_INSUFFICIENT_MEM ", };

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;
// Clock instances for internal periodic events.
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event to clock handler
mrClockEventData_t periodicUpdateData = { .event = MR_EVT_PERIODIC };

// Memory to pass RPA read event ID to clock handler
mrClockEventData_t argRpaRead = { .event = MR_EVT_READ_RPA };
// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task configuration
Task_Struct mrTask;

#ifndef FREERTOS
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(mrTaskStack, 8)
#else
#pragma data_alignment=8
#endif
#ifndef FREERTOS
uint8_t mrTaskStack[MR_TASK_STACK_SIZE];
#endif
#endif

// Maximim PDU size (default = 27 octets)
static uint16 mrMaxPduSize;

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
// Number of scan results filtered by Service UUID
static uint8_t numScanRes = 0;

// Scan results filtered by Service UUID
static scanRec_t scanList[DEFAULT_MAX_SCAN_RES];
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Value to write
static uint8_t charVal = 0;

// Number of connected devices
static uint8_t numConn = 0;

// Connection handle of current connection
static uint16_t mrConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Per-handle connection info
static mrConnRec_t connList[MAX_NUM_BLE_CONNS];

// Advertising handles
static uint8 advHandle;

static bool mrIsAdvertising = false;
// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = { 0 };

// Initiating PHY
static uint8_t mrInitPhy = INIT_PHY_1M;

// JUXTA
static Clock_Struct clkJuxtaLEDTimeout;
mrClockEventData_t argJuxtaLEDTimeout = { .event = JUXTA_EVT_LED_TIMEOUT };

typedef enum
{
    JUXTA_MODE_AXY_LOGGER, JUXTA_MODE_SHELF, JUXTA_MODE_ADVERTISE_NOSCAN
} juxtaMode_t;

static uint8_t juxtaMode = JUXTA_MODE_AXY_LOGGER; //JUXTA_MODE_SHELF; // init mode

NVS_Handle nvsHandle;
NVS_Attrs regionAttrs;
static uint32_t logCount = 0;
//static uint8_t nvsDataBuffer[JUXTA_LOG_SIZE];
static uint32_t nvsConfigBuffer[JUXTA_CONFIG_SIZE];
static uint32_t localTime = 0;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;

typedef struct
{
    uint_least8_t CS_PIN;
    uint_least8_t CLK_PIN;
    uint_least8_t DIO_PIN;
} sensbus_t;

static sensbus_t xl_bus = { CS_XL, SPC, SDIO };
static sensbus_t mag_bus = { CS_MAG, SPC, SDIO };
stmdev_ctx_t dev_ctx_xl;
stmdev_ctx_t dev_ctx_mg;

GPIO_PinConfig sdioPinConfigs[2] = { GPIO_CFG_OUTPUT_INTERNAL
                                             | GPIO_CFG_OUT_STR_MED
                                             | GPIO_CFG_OUT_LOW, /* OUTPUT */
                                     GPIO_CFG_INPUT_INTERNAL
                                             | GPIO_CFG_IN_INT_NONE
                                             | GPIO_CFG_PULL_NONE_INTERNAL, /* INPUT */
};

//bool juxtaRadio = false;
//uint8_t juxtaRadioCount = 0;

//UART_Handle uart = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void multi_role_init(void);
static void multi_role_scanInit(void);
static void multi_role_advertInit(void);
static void multi_role_taskFxn(UArg a0, UArg a1);

static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg);
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg);
static void multi_role_processAppMsg(mrEvt_t *pMsg);
static void multi_role_processCharValueChangeEvt(uint8_t paramID);
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void multi_role_processPasscode(mrPasscodeData_t *pData);
static void multi_role_processPairState(mrPairStateData_t *pairingEvent);
static void multi_role_processGapMsg(gapEventHdr_t *pMsg);
static void multi_role_processParamUpdate(uint16_t connHandle);
static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData);

static void multi_role_charValueChangeCB(uint8_t paramID);
static status_t multi_role_enqueueMsg(uint8_t event, void *pData);
static uint16_t multi_role_getConnIndex(uint16_t connHandle);
static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role);
static void multi_role_performPeriodicTask(void);
static void multi_role_clockHandler(UArg arg);
static uint8_t multi_role_clearConnListEntry(uint16_t connHandle);
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType);
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint16_t dataLen);
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID
static uint8_t multi_role_removeConnInfo(uint16_t connHandle);

static void multi_role_startSvcDiscovery(void);
#ifndef Display_DISABLE_ALL
static char* multi_role_getConnAddrStr(uint16_t connHandle);
#endif
static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg);
static void multi_role_scanCB(uint32_t evt, void *msg, uintptr_t arg);
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison);
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status);
static void multi_role_updateRPA(void);

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void shutdownLEDs(void);
static void toggleLED(uint8_t index);
static void blink(uint8_t runOnce);
static uint32_t rev32(uint32_t bytes);
static void recallNVS(void);
static void saveConfigs(void);

//static void platform_init(void);

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple GATT Profile Callbacks
static simpleProfileCBs_t multi_role_simpleProfileCBs = {
        multi_role_charValueChangeCB // Characteristic value change callback
        };

// GAP Bond Manager Callbacks
static gapBondCBs_t multi_role_BondMgrCBs = { multi_role_passcodeCB, // Passcode callback
        multi_role_pairStateCB                  // Pairing state callback
        };

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    // lsm303agr_read_reg(ctx, LSM303AGR_OFFSET_X_REG_L_M, buff, 6);
    sensbus_t *sensbus = (sensbus_t*) handle;
    uint8_t i;
    reg |= 0x80; // set read bit
    if (len > 1)
    {
        reg |= 0x40; // MS/increment address
    }

    shutdownLEDs(); // !!RM
    GPIO_write(sensbus->CLK_PIN, 1); // start high
    GPIO_write(sensbus->CS_PIN, 0); // active low
    // write addr
    for (i = 0; i < 8; i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        GPIO_write(LED1, 0); // !!RM

        uint8_t bit_value = ((reg >> (8 - (i - 1))) & 1);
        GPIO_write(sensbus->DIO_PIN, bit_value); // addr, MSB
        GPIO_write(LED2, bit_value); // !!RM

        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        GPIO_write(LED1, 1); // !!RM
        usleep(SPI_HALF_PERIOD); // delay again
    }
    // read data len
    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[1]); // set as input
    for (i = 0; i < (8 * len); i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        GPIO_write(LED1, 0); // !!RM

        bufp[(8 * len) - (i - 1)] = GPIO_read(sensbus->DIO_PIN);
        GPIO_write(LED2, GPIO_read(sensbus->DIO_PIN)); // !!RM

        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        GPIO_write(LED1, 1); // !!RM
        usleep(SPI_HALF_PERIOD); // delay again
    }
    GPIO_write(sensbus->CS_PIN, 1); // de-activate
    GPIO_setConfig(sensbus->DIO_PIN, sdioPinConfigs[0]); // set as output
    return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    sensbus_t *sensbus = (sensbus_t*) handle;
    uint8_t i;
    if (len > 1)
    {
        reg |= 0x40; // MS/increment address
    }

    GPIO_write(sensbus->CLK_PIN, 1); // start high
    GPIO_write(sensbus->CS_PIN, 0); // active low
    // write addr
    for (i = 0; i < 8; i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        uint8_t bit_value = ((reg >> (8 - (i - 1))) & 1);
        GPIO_write(sensbus->DIO_PIN, bit_value); // addr, MSB
        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        usleep(SPI_HALF_PERIOD); // delay again
    }
    // read data len
    for (i = 0; i < (8 * len); i++)
    {
        GPIO_write(sensbus->CLK_PIN, 0); // clock down
        GPIO_write(sensbus->DIO_PIN, bufp[(8 * len) - (i - 1)]);
        usleep(SPI_HALF_PERIOD); // delay half period
        GPIO_write(sensbus->CLK_PIN, 1); // clock up
        usleep(SPI_HALF_PERIOD); // delay again
    }
    GPIO_write(sensbus->CS_PIN, 1); // de-activate
    return 0;
}

static void platform_delay(uint32_t ms)
{
    usleep(ms * 1000);
}

static void toggleLED(uint8_t index)
{
    if (index == LED1 || index == LED2)
    {
        GPIO_write(index, 1);
        Util_startClock(&clkJuxtaLEDTimeout);
    }
}

static void shutdownLEDs(void)
{
    GPIO_write(LED1, 0);
    GPIO_write(LED2, 0);
}

static void blink(uint8_t runOnce)
{
    while (1)
    {
        GPIO_write(LED1, 1);
        usleep(50000);
        GPIO_write(LED1, 0);
        usleep(50000);
        if (runOnce == 1)
        {
            break;
        }
    }
}

static uint32_t rev32(uint32_t bytes)
{
    uint32_t aux = 0;
    uint8_t byte;
    int i;

    for (i = 0; i < 32; i += 8)
    {
        byte = (bytes >> i) & 0xff;
        aux |= byte << (32 - 8 - i);
    }
    return aux;
}

static void recallNVS(void)
{
    nvsHandle = NVS_open(NVS_JUXTA, NULL);
    if (nvsHandle != NULL)
    {
        NVS_read(nvsHandle, JUXTA_CONFIG_OFFSET_LOGCOUNT, (void*) &logCount,
                 sizeof(logCount));
        NVS_close(nvsHandle);
    }
}

static void saveConfigs(void)
{
    nvsHandle = NVS_open(NVS_JUXTA, NULL);
    if (nvsHandle != NULL)
    {
        NVS_getAttrs(nvsHandle, &regionAttrs);
        NVS_erase(nvsHandle, 0, regionAttrs.sectorSize); // erase all each time
        memcpy(nvsConfigBuffer + JUXTA_CONFIG_OFFSET_LOGCOUNT, &logCount,
               sizeof(logCount));
        NVS_write(nvsHandle, 0, (void*) nvsConfigBuffer,
                  sizeof(nvsConfigBuffer),
                  NVS_WRITE_POST_VERIFY);
        NVS_close(nvsHandle);

        uint32_t logCount_human = rev32(logCount);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN,
                                   &logCount_human);
    }
}

/*********************************************************************
 * @fn      multi_role_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void multi_role_spin(void)
{
    volatile uint8_t x = 0;

    while (1)
    {
        x++;
    }
}

/*********************************************************************
 * @fn      multi_role_createTask
 *
 * @brief   Task creation function for multi_role.
 *
 * @param   None.
 *
 * @return  None.
 */

void multi_role_createTask(void)
{
    Task_Params taskParams;
    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = mrTaskStack;
    taskParams.stackSize = MR_TASK_STACK_SIZE;
    taskParams.priority = MR_TASK_PRIORITY;
    Task_construct(&mrTask, multi_role_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      multi_role_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void multi_role_init(void)
{
    GPIO_init();
    GPIO_write(LED1, 1);

    // swap address to include BLE MAC address (unique for each device)
    uint64_t bleAddress = *((uint64_t*) (FCFG1_BASE + FCFG1_O_MAC_BLE_0))
            & 0xFFFFFFFFFFFF;
    char newAddress[GAP_DEVICE_NAME_LEN] = ""; // +1 for null
    sprintf(newAddress, "JX_%llX", bleAddress); // use <4 chars as prepend
    memcpy(attDeviceName, newAddress, GAP_DEVICE_NAME_LEN);

    NVS_init();
    recallNVS();

    SPI_init();

    dev_ctx_xl.write_reg = platform_write;
    dev_ctx_xl.read_reg = platform_read;
    dev_ctx_xl.handle = (void*) &xl_bus;
    dev_ctx_mg.write_reg = platform_write;
    dev_ctx_mg.read_reg = platform_read;
    dev_ctx_mg.handle = (void*) &mag_bus;
    /* Wait sensor boot time */
    platform_delay(LSM303AGR_BOOT_TIME);
    /* Check device ID */
    whoamI = 0;
    lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
//    spiHandle = MC3635_init(CONFIG_SPI);
//    if (spiHandle == NULL)
//    {
//        blink(0); // error
//    }
//    if (!MC3635_start(spiHandle))
//    {
//        blink(0); // error
//    }
//    modeCallback(juxtaMode); // resets sniff for JUXTA_MODE_AXY_LOGGER, stops sniff for others

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", MR_TASK_PRIORITY);
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &syncEvent);

//  Display_printf(dispHandle, MR_ROW_SEPARATOR, 0, "====================");

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);
    // Create one-shot clock for internal periodic events.
    Util_constructClock(&clkPeriodic, multi_role_clockHandler,
    MR_PERIODIC_EVT_PERIOD,
                        0, false, (UArg) &periodicUpdateData);

    Util_constructClock(&clkJuxtaLEDTimeout, multi_role_clockHandler,
    JUXTA_LED_TIMEOUT_PERIOD,
                        0, false, (UArg) &argJuxtaLEDTimeout);

    // Initialize Connection List
    multi_role_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

    // Set the Device Name characteristic in the GAP GATT Service
    // For more information, see the section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN,
                     (void* )attDeviceName);

    // Configure GAP
    {
        uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

        // Pass all parameter update requests to the app for it to decide
        GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
    }

    // Set default values for Data Length Extension
    // Extended Data Length Feature is already enabled by default
    {
        // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
        // Some brand smartphone is essentially needing 251/2120, so we set them here.
#define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
#define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

        // This API is documented in hci.h
        // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
        // http://software-dl.ti.com/lprf/ble5stack-latest/
        HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE,
                                               APP_SUGGESTED_TX_TIME);
    }

    // Initialize GATT Client, used by GAPBondMgr to look for RPAO characteristic for network privacy
#ifdef __GNUC__
    GATT_InitClient("");
#else
  GATT_InitClient();
#endif //__GNUC__

    // Register to receive incoming ATT Indications/Notifications
    GATT_RegisterForInd(selfEntity);

    // Setup the GAP Bond Manager
    setBondManagerParameters();

    // Initialize GATT attributes
    GGS_AddService(GAP_SERVICE);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service
    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Setup the SimpleProfile Characteristic Values
    // For more information, see the GATT and GATTServApp sections in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    {
        uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = { 0, 0, 0, 0 };
        uint32_t logCount_human = rev32(logCount);
        memcpy(charValue1, &logCount_human, sizeof(logCount_human));

        uint8_t charValue2[SIMPLEPROFILE_CHAR2_LEN] = { 0, 0, 0, 0 };
        uint32_t localTime_human = rev32(localTime);
        memcpy(charValue2, &localTime_human, sizeof(localTime_human));

        uint8_t charValue3 = juxtaMode;

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN,
                                   charValue1);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN,
                                   charValue2);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                                   &charValue3);
    }

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&multi_role_simpleProfileCBs);

    // Start Bond Manager and register callback
    VOID GAPBondMgr_Register(&multi_role_BondMgrCBs);

    // Register with GAP for HCI/Host messages. This is needed to receive HCI
    // events. For more information, see the HCI section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL);
    //Initialize GAP layer for Peripheral and Central role and register to receive GAP events
    GAP_DeviceInit(GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL, selfEntity,
                   addrMode, &pRandomAddress);

    shutdownLEDs();
//    if (GPIO_read(DEBUG) == 0)
//    {
//        dumpLog();
//    }
}

/*********************************************************************
 * @fn      multi_role_taskFxn
 *
 * @brief   Application task entry point for the multi_role.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void multi_role_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    multi_role_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread

        events = Event_pend(syncEvent, Event_Id_NONE, MR_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER); // event_31 + event_30

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void**) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8_t safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event*) pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature != 0xffff)
                    {
                        // Process inter-task message
                        safeToDealloc = multi_role_processStackMsg(
                                (ICall_Hdr*) pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            if (events & MR_QUEUE_EVT)
            {
                while (!Queue_empty(appMsgQueue))
                {
                    mrEvt_t *pMsg = (mrEvt_t*) Util_dequeueMsg(appMsgQueue);
                    if (pMsg)
                    {
                        // Process message.
                        multi_role_processAppMsg(pMsg);

                        // Free the space from the message.
                        ICall_free(pMsg);
                    }
                }
            }
        }
    }
}

/*********************************************************************
 * @fn      multi_role_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t multi_role_processStackMsg(ICall_Hdr *pMsg)
{
    uint8_t safeToDealloc = TRUE;

    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

    switch (pMsg->event)
    {
    case GAP_MSG_EVENT:
        //multi_role_processRoleEvent((gapMultiRoleEvent_t *)pMsg);
        multi_role_processGapMsg((gapEventHdr_t*) pMsg);
        break;

    case GATT_MSG_EVENT:
        // Process GATT message
        safeToDealloc = multi_role_processGATTMsg((gattMsgEvent_t*) pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {
        // Process HCI message
        switch (pMsg->status)
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
            break;

            // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
            hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t*) pMsg;
            switch (pMyMsg->cmdOpcode)
            {
            case HCI_LE_SET_PHY:
            {
                if (pMyMsg->cmdStatus ==
                HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
                {
//                    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
//                            "PHY Change failure, peer does not support this");
                }
                else
                {
//                    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
//                                   "PHY Update Status: 0x%02x",
//                                   pMyMsg->cmdStatus);
                }
            }
                break;
            case HCI_DISCONNECT:
                break;

            default:
            {
//                  Display_printf(dispHandle, MR_ROW_NON_CONN, 0,
//                                 "Unknown Cmd Status: 0x%04x::0x%02x",
//                                 pMyMsg->cmdOpcode, pMyMsg->cmdStatus);
            }
                break;
            }
        }
            break;

            // LE Events
        case HCI_LE_EVENT_CODE:
        {
            hciEvt_BLEPhyUpdateComplete_t *pPUC =
                    (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

            if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
            {
                if (pPUC->status != SUCCESS)
                {

//              Display_printf(dispHandle, MR_ROW_ANY_CONN, 0,
//                             "%s: PHY change failure",
//                             multi_role_getConnAddrStr(pPUC->connHandle));
                }
                else
                {
//              Display_printf(dispHandle, MR_ROW_ANY_CONN, 0,
//                             "%s: PHY updated to %s",
//                             multi_role_getConnAddrStr(pPUC->connHandle),
//              // Only symmetrical PHY is supported.
//              // rxPhy should be equal to txPhy.
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1 Mbps" :
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2 Mbps" :
//                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "Coded" : "Unexpected PHY Value");
                }
            }

            break;
        }

        default:
            break;
        }

        break;
    }

    case L2CAP_SIGNAL_EVENT:
        // place holder for L2CAP Connection Parameter Reply
        break;

    default:
        // Do nothing
        break;
    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      multi_role_processGapMsg
 *
 * @brief   GAP message processing function.
 *
 * @param   pMsg - pointer to event message structure
 *
 * @return  none
 */
static void multi_role_processGapMsg(gapEventHdr_t *pMsg)
{
    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        if (pPkt->hdr.status == SUCCESS)
        {
            // Store the system ID
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = pPkt->devAddr[0];
            systemId[1] = pPkt->devAddr[1];
            systemId[2] = pPkt->devAddr[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = pPkt->devAddr[5];
            systemId[6] = pPkt->devAddr[4];
            systemId[5] = pPkt->devAddr[3];

            // Set Device Info Service Parameter
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);

            BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- start advert %d,%d\n", 0, 0);
            //Setup and start advertising
            multi_role_advertInit();

        }

        //Setup scanning
        multi_role_scanInit();

        mrMaxPduSize = pPkt->dataPktLen;

        //Display initialized state status
//      Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);
//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Initialized");
//      Display_printf(dispHandle, MR_ROW_MYADDRSS, 0, "Multi-Role Address: %s",(char *)Util_convertBdAddr2Str(pPkt->devAddr));

        break;
    }

    case GAP_CONNECTING_CANCELLED_EVENT:
    {
//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0,
//                     "Connecting attempt cancelled");
        break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        uint16_t connHandle = ((gapEstLinkReqEvent_t*) pMsg)->connectionHandle;
        uint8_t role = ((gapEstLinkReqEvent_t*) pMsg)->connRole;
        uint8_t *pAddr = ((gapEstLinkReqEvent_t*) pMsg)->devAddr;
        uint8_t connIndex;
        uint8_t *pStrAddr;

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
        // Add this connection info to the list
        connIndex = multi_role_addConnInfo(connHandle, pAddr, role);

        // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

        connList[connIndex].charHandle = 0;

        Util_startClock(&clkPeriodic);

        pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Connected to %s", pStrAddr);
//      Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);

        if (numConn < MAX_NUM_BLE_CONNS)
        {
            // Start advertising since there is room for more connections
            GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);
        }
        else
        {
            // Stop advertising since there is no room for more connections
            GapAdv_disable(advHandle);
        }

        break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
        uint16_t connHandle =
                ((gapTerminateLinkEvent_t*) pMsg)->connectionHandle;
        uint8_t connIndex;
        uint8_t *pStrAddr;

        BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
        // Mark this connection deleted in the connected device list.
        connIndex = multi_role_removeConnInfo(connHandle);

        // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
        MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

        pStrAddr = (uint8_t*) Util_convertBdAddr2Str(connList[connIndex].addr);

//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "%s is disconnected",
//                     pStrAddr);
//      Display_printf(dispHandle, MR_ROW_NUM_CONN, 0, "Num Conns: %d", numConn);

        // Start advertising since there is room for more connections
        GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);

        // If no active connections
        if (numConn == 0)
        {
            // Stop periodic clock
            Util_stopClock(&clkPeriodic);
        }

        break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
        gapUpdateLinkParamReqReply_t rsp;
        gapUpdateLinkParamReqEvent_t *pReq =
                (gapUpdateLinkParamReqEvent_t*) pMsg;

        rsp.connectionHandle = pReq->req.connectionHandle;
        rsp.signalIdentifier = pReq->req.signalIdentifier;

        // Only accept connection intervals with slave latency of 0
        // This is just an example of how the application can send a response
        if (pReq->req.connLatency == 0)
        {
            rsp.intervalMin = pReq->req.intervalMin;
            rsp.intervalMax = pReq->req.intervalMax;
            rsp.connLatency = pReq->req.connLatency;
            rsp.connTimeout = pReq->req.connTimeout;
            rsp.accepted = TRUE;
        }
        else
        {
            rsp.accepted = FALSE;
        }

        // Send Reply
        VOID GAP_UpdateLinkParamReqReply(&rsp);

        break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t*) pMsg;

        // Get the address from the connection handle
        linkDBInfo_t linkInfo;
        if (linkDB_GetInfo(pPkt->connectionHandle, &linkInfo) == SUCCESS)
        {

            if (pPkt->status == SUCCESS)
            {
//            Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
//                          "Updated: %s, connTimeout:%d",
//                           Util_convertBdAddr2Str(linkInfo.addr),
//                           linkInfo.connTimeout*CONN_TIMEOUT_MS_CONVERSION);
            }
            else
            {
                // Display the address of the connection update failure
//            Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
//                           "Update Failed 0x%h: %s", pPkt->opcode,
//                           Util_convertBdAddr2Str(linkInfo.addr));
            }
        }
        // Check if there are any queued parameter updates
        mrConnHandleEntry_t *connHandleEntry = (mrConnHandleEntry_t*) List_get(
                &paramUpdateList);
        if (connHandleEntry != NULL)
        {
            // Attempt to send queued update now
            multi_role_processParamUpdate(connHandleEntry->connHandle);

            // Free list element
            ICall_free(connHandleEntry);
        }
        break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
     case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
     {
       linkDBInfo_t linkInfo;
       gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

       // Get the address from the connection handle
       linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

       // Display the address of the connection update failure
//       Display_printf(dispHandle, MR_ROW_CUR_CONN, 0,
//                      "Peer Device's Update Request Rejected 0x%h: %s", pPkt->opcode,
//                      Util_convertBdAddr2Str(linkInfo.addr));

       break;
     }
#endif

    default:
        break;
    }
}

/*********************************************************************
 * @fn      multi_role_scanInit
 *
 * @brief   Setup initial device scan settings.
 *
 * @return  None.
 */
static void multi_role_scanInit(void)
{
    uint8_t temp8;
    uint16_t temp16;

    // Setup scanning
    // For more information, see the GAP section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/

    // Register callback to process Scanner events
    GapScan_registerCb(multi_role_scanCB, NULL);

    // Set Scanner Event Mask
    GapScan_setEventMask(
            GAP_EVT_SCAN_ENABLED | GAP_EVT_SCAN_DISABLED | GAP_EVT_ADV_REPORT);

    // Set Scan PHY parameters
    GapScan_setPhyParams(DEFAULT_SCAN_PHY, DEFAULT_SCAN_TYPE,
                         DEFAULT_SCAN_INTERVAL, DEFAULT_SCAN_WINDOW);

    // Set Advertising report fields to keep
    temp16 = ADV_RPT_FIELDS;
    GapScan_setParam(SCAN_PARAM_RPT_FIELDS, &temp16);
    // Set Scanning Primary PHY
    temp8 = DEFAULT_SCAN_PHY;
    GapScan_setParam(SCAN_PARAM_PRIM_PHYS, &temp8);
    // Set LL Duplicate Filter
    temp8 = SCANNER_DUPLICATE_FILTER;
    GapScan_setParam(SCAN_PARAM_FLT_DUP, &temp8);

    // Set PDU type filter -
    // Only 'Connectable' and 'Complete' packets are desired.
    // It doesn't matter if received packets are
    // whether Scannable or Non-Scannable, whether Directed or Undirected,
    // whether Scan_Rsp's or Advertisements, and whether Legacy or Extended.
    temp16 = SCAN_FLT_PDU_CONNECTABLE_ONLY | SCAN_FLT_PDU_COMPLETE_ONLY;
    GapScan_setParam(SCAN_PARAM_FLT_PDU_TYPE, &temp16);

    // Set initiating PHY parameters
    GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MIN,
                        INIT_PHYPARAM_MIN_CONN_INT);
    GapInit_setPhyParam(DEFAULT_INIT_PHY, INIT_PHYPARAM_CONN_INT_MAX,
                        INIT_PHYPARAM_MAX_CONN_INT);
}

/*********************************************************************
 * @fn      multi_role_advertInit
 *
 * @brief   Setup initial advertisment and start advertising from device init.
 *
 * @return  None.
 */
static void multi_role_advertInit(void)
{
    uint8_t status = FAILURE;
    // Setup and start Advertising
    // For more information, see the GAP section in the User's Guide:
    // http://software-dl.ti.com/lprf/ble5stack-latest/

    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
    // Create Advertisement set #1 and assign handle
    GapAdv_create(&multi_role_advCB, &advParams1, &advHandle);

    // Load advertising data for set #1 that is statically allocated by the app
    GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_ADV, sizeof(advData1),
                        advData1);

    // Load scan response data for set #1 that is statically allocated by the app
    GapAdv_loadByHandle(advHandle, GAP_ADV_DATA_TYPE_SCAN_RSP,
                        sizeof(scanResData1), scanResData1);

    // Set event mask for set #1
    GapAdv_setEventMask(
            advHandle,
            GAP_ADV_EVT_MASK_START_AFTER_ENABLE
                    | GAP_ADV_EVT_MASK_END_AFTER_DISABLE
                    | GAP_ADV_EVT_MASK_SET_TERMINATED);

    BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
    // Enable legacy advertising for set #1
    status = GapAdv_enable(advHandle, GAP_ADV_ENABLE_OPTIONS_USE_MAX, 0);

    if (status != SUCCESS)
    {
        mrIsAdvertising = false;
//    Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Error: Failed to Start Advertising!");
    }

    if (addrMode > ADDRMODE_RANDOM)
    {
        multi_role_updateRPA();
        // Create one-shot clock for RPA check event.
        Util_constructClock(&clkRpaRead, multi_role_clockHandler,
        READ_RPA_PERIOD,
                            0, true, (UArg) &argRpaRead);
    }
}

/*********************************************************************
 * @fn      multi_role_advCB
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void multi_role_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
    mrGapAdvEventData_t *pData = ICall_malloc(sizeof(mrGapAdvEventData_t));

    if (pData)
    {
        pData->event = event;
        pData->pBuf = pBuf;

        if (multi_role_enqueueMsg(MR_EVT_ADV, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t multi_role_processGATTMsg(gattMsgEvent_t *pMsg)
{
    // Get connection index from handle
    uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
//    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
//    Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
    }

    // Messages from GATT server
    if (linkDB_Up(pMsg->connHandle))
    {
        if ((pMsg->method == ATT_READ_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
        {
            if (pMsg->method == ATT_ERROR_RSP)
            {
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a successful read, display the read value
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Read rsp: %d", pMsg->msg.readRsp.pValue[0]);
            }

        }
        else if ((pMsg->method == ATT_WRITE_RSP)
                || ((pMsg->method == ATT_ERROR_RSP)
                        && (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
        {

            if (pMsg->method == ATT_ERROR_RSP)
            {
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write Error %d", pMsg->msg.errorRsp.errCode);
            }
            else
            {
                // After a succesful write, display the value that was written and
                // increment value
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Write sent: %d", charVal);
            }
        }
        else if (connList[connIndex].discState != BLE_DISC_STATE_IDLE)
        {
            multi_role_processGATTDiscEvent(pMsg);
        }
    } // Else - in case a GATT message came after a connection has dropped, ignore it.

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

/*********************************************************************
 * @fn		multi_role_processParamUpdate
 *
 * @brief	Process connection parameters update
 *
 * @param	connHandle - connection handle to update
 *
 * @return	None.
 */
static void multi_role_processParamUpdate(uint16_t connHandle)
{
    gapUpdateLinkParamReq_t req;
    uint8_t connIndex;

    req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
    req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

    connIndex = multi_role_getConnIndex(connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);
    // Free clock struct, only in case it is not NULL
    if (connList[connIndex].pUpdateClock != NULL)
    {
        ICall_free(connList[connIndex].pUpdateClock);
        connList[connIndex].pUpdateClock = NULL;
    }
    // Free ParamUpdateEventData, only in case it is not NULL
    if (connList[connIndex].pParamUpdateEventData != NULL)
    {
        ICall_free(connList[connIndex].pParamUpdateEventData);
    }

    // Send parameter update
    bStatus_t status = GAP_UpdateLinkParamReq(&req);

    // If there is an ongoing update, queue this for when the udpate completes
    if (status == bleAlreadyInRequestedMode)
    {
        mrConnHandleEntry_t *connHandleEntry = ICall_malloc(
                sizeof(mrConnHandleEntry_t));
        if (connHandleEntry)
        {
            connHandleEntry->connHandle = connHandle;

            List_put(&paramUpdateList, (List_Elem*) connHandleEntry);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void multi_role_processAppMsg(mrEvt_t *pMsg)
{
    bool safeToDealloc = TRUE;

    if (pMsg->event <= APP_EVT_EVENT_MAX)
    {
        BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
    }
    else
    {
        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
    }

    switch (pMsg->event)
    {
    case MR_EVT_CHAR_CHANGE:
    {
        multi_role_processCharValueChangeEvt(*(uint8_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_ADV_REPORT:
    {
        GapScan_Evt_AdvRpt_t *pAdvRpt = (GapScan_Evt_AdvRpt_t*) (pMsg->pData);

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        if (multi_role_findSvcUuid(SIMPLEPROFILE_SERV_UUID, pAdvRpt->pData,
                                   pAdvRpt->dataLen))
        {
            multi_role_addScanInfo(pAdvRpt->addr, pAdvRpt->addrType);
//        Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Discovered: %s",
//                       Util_convertBdAddr2Str(pAdvRpt->addr));
        }
        if (multi_role_findSvcUuid(TIME_SERVICE_UUID, pAdvRpt->pData,
                                   pAdvRpt->dataLen))
        {
            uint8_t svcLoc;
            uint8_t newTime[8] = { 0 };
            for (svcLoc = 0; svcLoc < pAdvRpt->dataLen - 12; svcLoc++)
            {
                if (pAdvRpt->pData[svcLoc] == LO_UINT16(TIME_SERVICE_UUID)
                        && pAdvRpt->pData[svcLoc + 1]
                                == HI_UINT16(TIME_SERVICE_UUID))
                {
                    memcpy(newTime, pAdvRpt->pData + svcLoc + 4,
                           sizeof(newTime));
                    localTime = strtol((char*) newTime, NULL, 16);
                    blink(true);
                    blink(true);
                    blink(true);
                    break;
                }
            }
        }
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
//      Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Discovered: %s",
//                     Util_convertBdAddr2Str(pAdvRpt->addr));
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

        // Free scan payload data
        if (pAdvRpt->pData != NULL)
        {
            ICall_free(pAdvRpt->pData);
        }
        break;
    }

    case MR_EVT_SCAN_ENABLED:
    {
//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Discovering...");
        break;
    }

    case MR_EVT_SCAN_DISABLED:
    {
        uint8_t numReport;
        uint8_t i;
        static uint8_t *pAddrs = NULL;
        uint8_t *pAddrTemp;
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        numReport = numScanRes;
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
      GapScan_Evt_AdvRpt_t advRpt;

      numReport = ((GapScan_Evt_End_t*) (pMsg->pData))->numReport;
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

//      Display_printf(dispHandle, MR_ROW_NON_CONN, 0,
//                     "%d devices discovered", numReport);

        if (pAddrs != NULL)
        {
            ICall_free(pAddrs);
        }
        // Allocate buffer to display addresses
        pAddrs = ICall_malloc(numReport * MR_ADDR_STR_SIZE);
        if (pAddrs == NULL)
        {
            numReport = 0;
        }

        if (pAddrs != NULL)
        {
            pAddrTemp = pAddrs;
            for (i = 0; i < numReport; i++, pAddrTemp += MR_ADDR_STR_SIZE)
            {
#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
                // Get the address from the list, convert it to string, and
                // copy the string to the address buffer
                memcpy(pAddrTemp, Util_convertBdAddr2Str(scanList[i].addr),
                MR_ADDR_STR_SIZE);
#else // !DEFAULT_DEV_DISC_BY_SVC_UUID
          // Get the address from the report, convert it to string, and
          // copy the string to the address buffer
          GapScan_getAdvReport(i, &advRpt);
          memcpy(pAddrTemp, Util_convertBdAddr2Str(advRpt.addr),
                 MR_ADDR_STR_SIZE);
  #endif // DEFAULT_DEV_DISC_BY_SVC_UUID
            }
        }
        break;
    }

    case MR_EVT_SVC_DISC:
    {
        multi_role_startSvcDiscovery();
        break;
    }

    case MR_EVT_ADV:
    {
        multi_role_processAdvEvent((mrGapAdvEventData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_PAIRING_STATE:
    {
        multi_role_processPairState((mrPairStateData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_PASSCODE_NEEDED:
    {
        multi_role_processPasscode((mrPasscodeData_t*) (pMsg->pData));
        break;
    }

    case MR_EVT_SEND_PARAM_UPDATE:
    {
        // Extract connection handle from data
        uint16_t locConnHandle =
                *(uint16_t*) (((mrClockEventData_t*) pMsg->pData)->data);
        multi_role_processParamUpdate(locConnHandle);
        safeToDealloc = FALSE;
        break;
    }

    case MR_EVT_PERIODIC:
    {
        multi_role_performPeriodicTask();
        break;
    }

    case MR_EVT_READ_RPA:
    {
        multi_role_updateRPA();
        break;
    }

    case JUXTA_EVT_LED_TIMEOUT:
    {
        shutdownLEDs();
        break;
    }

    case MR_EVT_INSUFFICIENT_MEM:
    {
        // We are running out of memory.
//      Display_printf(dispHandle, MR_ROW_ANY_CONN, 0, "Insufficient Memory");

        // We might be in the middle of scanning, try stopping it.
#ifdef __GNUC__
        GapScan_disable("");
#else
      GapScan_disable();
#endif //__GNUC__
        break;
    }

    default:
        // Do nothing.
        break;
    }

    if ((safeToDealloc == TRUE) && (pMsg->pData != NULL))
    {
        ICall_free(pMsg->pData);
    }
}

/*********************************************************************
 * @fn      multi_role_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void multi_role_processAdvEvent(mrGapAdvEventData_t *pEventData)
{
    switch (pEventData->event)
    {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
        mrIsAdvertising = true;
//      Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
        break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
        mrIsAdvertising = false;
//      Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
        break;

    case GAP_EVT_ADV_START:
//      Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Started %d Enabled",
//                     *(uint8_t *)(pEventData->pBuf));
        break;

    case GAP_EVT_ADV_END:
//      Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Ended %d Disabled",
//                     *(uint8_t *)(pEventData->pBuf));
        break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
        mrIsAdvertising = false;
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
#endif
//      Display_printf(dispHandle, MR_ROW_ADVERTIS, 0, "Adv Set %d disabled after conn %d",
//                     advSetTerm->handle, advSetTerm->connHandle );
    }
        break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
        break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
        break;

    default:
        break;
    }

    // All events have associated memory to free except the insufficient memory
    // event
    if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
    {
        ICall_free(pEventData->pBuf);
    }
}

#if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
/*********************************************************************
 * @fn      multi_role_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool multi_role_findSvcUuid(uint16_t uuid, uint8_t *pData,
                                   uint16_t dataLen)
{
    uint8_t adLen;
    uint8_t adType;
    uint8_t *pEnd;

    if (dataLen > 0)
    {
        pEnd = pData + dataLen - 1;

        // While end of data not reached
        while (pData < pEnd)
        {
            // Get length of next AD item
            adLen = *pData++;
            if (adLen > 0)
            {
                adType = *pData;

                // If AD type is for 16-bit service UUID
                if ((adType == GAP_ADTYPE_16BIT_MORE)
                        || (adType == GAP_ADTYPE_16BIT_COMPLETE))
                {
                    pData++;
                    adLen--;

                    // For each UUID in list
                    while (adLen >= 2 && pData < pEnd)
                    {
                        // Check for match
                        if ((pData[0] == LO_UINT16(uuid))
                                && (pData[1] == HI_UINT16(uuid)))
                        {
                            // Match found
                            return TRUE;
                        }

                        // Go to next
                        pData += 2;
                        adLen -= 2;
                    }

                    // Handle possible erroneous extra byte in UUID list
                    if (adLen == 1)
                    {
                        pData++;
                    }
                }
                else
                {
                    // Go to next item
                    pData += adLen;
                }
            }
        }
    }

    // Match not found
    return FALSE;
}

/*********************************************************************
 * @fn      multi_role_addScanInfo
 *
 * @brief   Add a device to the scanned device list
 *
 * @return  none
 */
static void multi_role_addScanInfo(uint8_t *pAddr, uint8_t addrType)
{
    uint8_t i;

    // If result count not at max
    if (numScanRes < DEFAULT_MAX_SCAN_RES)
    {
        // Check if device is already in scan results
        for (i = 0; i < numScanRes; i++)
        {
            if (memcmp(pAddr, scanList[i].addr, B_ADDR_LEN) == 0)
            {
                return;
            }
        }

        // Add addr to scan result list
        memcpy(scanList[numScanRes].addr, pAddr, B_ADDR_LEN);
        scanList[numScanRes].addrType = addrType;

        // Increment scan result count
        numScanRes++;
    }
}
#endif // DEFAULT_DEV_DISC_BY_SVC_UUID

/*********************************************************************
 * @fn      multi_role_scanCB
 *
 * @brief   Callback called by GapScan module
 *
 * @param   evt - event
 * @param   msg - message coming with the event
 * @param   arg - user argument
 *
 * @return  none
 */
void multi_role_scanCB(uint32_t evt, void *pMsg, uintptr_t arg)
{
    uint8_t event;

    if (evt & GAP_EVT_ADV_REPORT)
    {
        event = MR_EVT_ADV_REPORT;
    }
    else if (evt & GAP_EVT_SCAN_ENABLED)
    {
        event = MR_EVT_SCAN_ENABLED;
    }
    else if (evt & GAP_EVT_SCAN_DISABLED)
    {
        event = MR_EVT_SCAN_DISABLED;
    }
    else if (evt & GAP_EVT_INSUFFICIENT_MEMORY)
    {
        event = MR_EVT_INSUFFICIENT_MEM;
    }
    else
    {
        return;
    }

    if (multi_role_enqueueMsg(event, pMsg) != SUCCESS)
    {
        ICall_free(pMsg);
    }

}

/*********************************************************************
 * @fn      multi_role_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void multi_role_charValueChangeCB(uint8_t paramID)
{
    uint8_t *pData;

    // Allocate space for the event data.
    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = paramID;

        // Queue the event.
        if (multi_role_enqueueMsg(MR_EVT_CHAR_CHANGE, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static status_t multi_role_enqueueMsg(uint8_t event, void *pData)
{
    uint8_t success;
    mrEvt_t *pMsg = ICall_malloc(sizeof(mrEvt_t));

    // Create dynamic pointer to message.
    if (pMsg)
    {
        pMsg->event = event;
        pMsg->pData = pData;

        // Enqueue the message.
        success = Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t*) pMsg);
        return (success) ? SUCCESS : FAILURE;
    }

    return (bleMemAllocError);
}

/*********************************************************************
 * @fn      multi_role_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void multi_role_processCharValueChangeEvt(uint8_t paramId)
{
    uint8_t newValue;

    switch (paramId)
    {
    case SIMPLEPROFILE_CHAR1:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

//      Display_printf(dispHandle, MR_ROW_CHARSTAT, 0, "Char 1: %d", (uint16_t)newValue);
        break;

    case SIMPLEPROFILE_CHAR3:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

//      Display_printf(dispHandle, MR_ROW_CHARSTAT, 0, "Char 3: %d", (uint16_t)newValue);
        break;

    default:
        // should not reach here!
        break;
    }
}

/*********************************************************************
 * @fn      multi_role_performPeriodicTask
 *
 * @brief   Now used for Juxta
 *
 * @param   None.
 *
 * @return  None.
 */
static void multi_role_performPeriodicTask(void)
{
    GPIO_toggle(LED1);
}

/*********************************************************************
 * @fn      multi_role_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void multi_role_updateRPA(void)
{
    uint8_t *pRpaNew;

    // Read the current RPA.
    pRpaNew = GAP_GetDevAddress(FALSE);

    if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
    {
        // If the RPA has changed, update the display
//    Display_printf(dispHandle, MR_ROW_RPA, 0, "RP Addr: %s",
//                   Util_convertBdAddr2Str(pRpaNew));
        memcpy(rpa, pRpaNew, B_ADDR_LEN);
    }
}

/*********************************************************************
 * @fn      multi_role_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void multi_role_clockHandler(UArg arg)
{
    mrClockEventData_t *pData = (mrClockEventData_t*) arg;

    if (pData->event == MR_EVT_PERIODIC)
    {
        // Start the next period
        Util_startClock(&clkPeriodic);

        // Send message to perform periodic task
        multi_role_enqueueMsg(MR_EVT_PERIODIC, NULL);
    }
    else if (pData->event == MR_EVT_READ_RPA)
    {
        // Start the next period
        Util_startClock(&clkRpaRead);

        // Send message to read the current RPA
        multi_role_enqueueMsg(MR_EVT_READ_RPA, NULL);
    }
    else if (pData->event == MR_EVT_SEND_PARAM_UPDATE)
    {
        // Send message to app
        multi_role_enqueueMsg(MR_EVT_SEND_PARAM_UPDATE, pData);
    }
}

/*********************************************************************
 * @fn      multi_role_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @param   pMsg - pointer to discovery event stack message
 *
 * @return  none
 */
static void multi_role_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{
    uint8_t connIndex = multi_role_getConnIndex(pMsg->connHandle);
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    if (connList[connIndex].discState == BLE_DISC_STATE_MTU)
    {
        // MTU size response received, discover simple service
        if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
        {
            uint8_t uuid[ATT_BT_UUID_SIZE] =
                    { LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(
                            SIMPLEPROFILE_SERV_UUID) };

            connList[connIndex].discState = BLE_DISC_STATE_SVC;

            // Discovery simple service
            VOID GATT_DiscPrimaryServiceByUUID(pMsg->connHandle, uuid,
                                               ATT_BT_UUID_SIZE, selfEntity);
        }
    }
    else if (connList[connIndex].discState == BLE_DISC_STATE_SVC)
    {
        // Service found, store handles
        if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP
                && pMsg->msg.findByTypeValueRsp.numInfo > 0)
        {
            svcStartHdl = ATT_ATTR_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
            svcEndHdl = ATT_GRP_END_HANDLE(
                    pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
        }

        // If procedure complete
        if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP)
                && (pMsg->hdr.status == bleProcedureComplete))
                || (pMsg->method == ATT_ERROR_RSP))
        {
            if (svcStartHdl != 0)
            {
                attReadByTypeReq_t req;

                // Discover characteristic
                connList[connIndex].discState = BLE_DISC_STATE_CHAR;

                req.startHandle = svcStartHdl;
                req.endHandle = svcEndHdl;
                req.type.len = ATT_BT_UUID_SIZE;
                req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

                VOID GATT_DiscCharsByUUID(pMsg->connHandle, &req, selfEntity);
            }
        }
    }
    else if (connList[connIndex].discState == BLE_DISC_STATE_CHAR)
    {
        // Characteristic found, store handle
        if ((pMsg->method == ATT_READ_BY_TYPE_RSP)
                && (pMsg->msg.readByTypeRsp.numPairs > 0))
        {
            uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

            // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
            MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

            // Store the handle of the simpleprofile characteristic 1 value
            connList[connIndex].charHandle = BUILD_UINT16(
                    pMsg->msg.readByTypeRsp.pDataList[3],
                    pMsg->msg.readByTypeRsp.pDataList[4]);

//      Display_printf(dispHandle, MR_ROW_CUR_CONN, 0, "Simple Svc Found");
            // Now we can use GATT Read/Write
        }

        connList[connIndex].discState = BLE_DISC_STATE_IDLE;
    }
}

/*********************************************************************
 * @fn      multi_role_getConnIndex
 *
 * @brief   Translates connection handle to index
 *
 * @param   connHandle - the connection handle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint16_t multi_role_getConnIndex(uint16_t connHandle)
{
    uint8_t i;
    // Loop through connection
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        // If matching connection handle found
        if (connList[i].connHandle == connHandle)
        {
            return i;
        }
    }

    // Not found if we got here
    return (MAX_NUM_BLE_CONNS);
}

#ifndef Display_DISABLE_ALL
/*********************************************************************
 * @fn      multi_role_getConnAddrStr
 *
 * @brief   Return, in string form, the address of the peer associated with
 *          the connHandle.
 *
 * @return  A null-terminated string of the address.
 *          if there is no match, NULL will be returned.
 */
static char* multi_role_getConnAddrStr(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return Util_convertBdAddr2Str(connList[i].addr);
    }
  }

  return NULL;
}
#endif

/*********************************************************************
 * @fn      multi_role_clearConnListEntry
 *
 * @brief   clear device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t multi_role_clearConnListEntry(uint16_t connHandle)
{
    uint8_t i;
    // Set to invalid connection index initially
    uint8_t connIndex = MAX_NUM_BLE_CONNS;

    if (connHandle != LINKDB_CONNHANDLE_ALL)
    {
        connIndex = multi_role_getConnIndex(connHandle);
        // Get connection index from handle
        if (connIndex >= MAX_NUM_BLE_CONNS)
        {
            return bleInvalidRange;
        }
    }

    // Clear specific handle or all handles
    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if ((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
        {
            connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
            connList[i].charHandle = 0;
            connList[i].discState = 0;
        }
    }

    return SUCCESS;
}

/************************************************************************
 * @fn      multi_role_pairStateCB
 *
 * @param   connHandle - the connection handle
 *
 * @param   state - pairing state
 *
 * @param   status - status of pairing state
 *
 * @return  none
 */
static void multi_role_pairStateCB(uint16_t connHandle, uint8_t state,
                                   uint8_t status)
{
    mrPairStateData_t *pData = ICall_malloc(sizeof(mrPairStateData_t));

    // Allocate space for the event data.
    if (pData)
    {
        pData->state = state;
        pData->connHandle = connHandle;
        pData->status = status;

        // Queue the event.
        if (multi_role_enqueueMsg(MR_EVT_PAIRING_STATE, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @param   deviceAddr - pointer to device address
 *
 * @param   connHandle - the connection handle
 *
 * @param   uiInputs - pairing User Interface Inputs
 *
 * @param   uiOutputs - pairing User Interface Outputs
 *
 * @param   numComparison - numeric Comparison 20 bits
 *
 * @return  none
 */
static void multi_role_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                  uint8_t uiInputs, uint8_t uiOutputs,
                                  uint32_t numComparison)
{
    mrPasscodeData_t *pData = ICall_malloc(sizeof(mrPasscodeData_t));

    // Allocate space for the passcode event.
    if (pData)
    {
        pData->connHandle = connHandle;
        memcpy(pData->deviceAddr, deviceAddr, B_ADDR_LEN);
        pData->uiInputs = uiInputs;
        pData->uiOutputs = uiOutputs;
        pData->numComparison = numComparison;

        // Enqueue the event.
        if (multi_role_enqueueMsg(MR_EVT_PASSCODE_NEEDED, pData) != SUCCESS)
        {
            ICall_free(pData);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @param   pairingEvent - pairing event received from the stack
 *
 * @return  none
 */
static void multi_role_processPairState(mrPairStateData_t *pPairData)
{
    uint8_t state = pPairData->state;
    uint8_t status = pPairData->status;

    switch (state)
    {
    case GAPBOND_PAIRING_STATE_STARTED:
//      Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing started");
        break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
        if (status == SUCCESS)
        {
            linkDBInfo_t linkInfo;

//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing success");

            if (linkDB_GetInfo(pPairData->connHandle, &linkInfo) == SUCCESS)
            {
                // If the peer was using private address, update with ID address
                if ((linkInfo.addrType == ADDRTYPE_PUBLIC_ID
                        || linkInfo.addrType == ADDRTYPE_RANDOM_ID)
                        && !Util_isBufSet(linkInfo.addrPriv, 0, B_ADDR_LEN))

                {
                    // Update the address of the peer to the ID address
//            Display_printf(dispHandle, MR_ROW_NON_CONN, 0, "Addr updated: %s",
//                           Util_convertBdAddr2Str(linkInfo.addr));

                    // Update the connection list with the ID address
                    uint8_t i = multi_role_getConnIndex(pPairData->connHandle);

                    MULTIROLE_ASSERT(i < MAX_NUM_BLE_CONNS);
                    memcpy(connList[i].addr, linkInfo.addr, B_ADDR_LEN);
                }
            }
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Pairing fail: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
        if (status == SUCCESS)
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption success");
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Encryption failed: %d", status);
        }
        break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
        if (status == SUCCESS)
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save success");
        }
        else
        {
//        Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Bond save failed: %d", status);
        }

        break;

    default:
        break;
    }
}

/*********************************************************************
 * @fn      multi_role_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void multi_role_processPasscode(mrPasscodeData_t *pData)
{
    // Display passcode to user
    if (pData->uiOutputs != 0)
    {
//    Display_printf(dispHandle, MR_ROW_SECURITY, 0, "Passcode: %d",
//                   B_APP_DEFAULT_PASSCODE);
    }

    // Send passcode response
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      multi_role_startSvcDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void multi_role_startSvcDiscovery(void)
{
    uint8_t connIndex = multi_role_getConnIndex(mrConnHandle);

    // connIndex cannot be equal to or greater than MAX_NUM_BLE_CONNS
    MULTIROLE_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

    attExchangeMTUReq_t req;

    // Initialize cached handles
    svcStartHdl = svcEndHdl = 0;

    connList[connIndex].discState = BLE_DISC_STATE_MTU;

    // Discover GATT Server's Rx MTU size
    req.clientRxMTU = mrMaxPduSize - L2CAP_HDR_SIZE;

    // ATT MTU size should be set to the minimum of the Client Rx MTU
    // and Server Rx MTU values
    VOID GATT_ExchangeMTU(mrConnHandle, &req, selfEntity);
}

/*********************************************************************
 * @fn      multi_role_addConnInfo
 *
 * @brief   add a new connection to the index-to-connHandle map
 *
 * @param   connHandle - the connection handle
 *
 * @param   addr - pointer to device address
 *
 * @return  index of connection handle
 */
static uint8_t multi_role_addConnInfo(uint16_t connHandle, uint8_t *pAddr,
                                      uint8_t role)
{
    uint8_t i;

    for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
    {
        if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
        {
            // Found available entry to put a new connection info in
            connList[i].connHandle = connHandle;
            memcpy(connList[i].addr, pAddr, B_ADDR_LEN);
            numConn++;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
            // If a peripheral, start the clock to send a connection parameter update
            if (role == GAP_PROFILE_PERIPHERAL)
            {
                // Allocate data to send through clock handler
                connList[i].pParamUpdateEventData = ICall_malloc(
                        sizeof(mrClockEventData_t) + sizeof(uint16_t));
                if (connList[i].pParamUpdateEventData)
                {
                    // Set clock data
                    connList[i].pParamUpdateEventData->event =
                    MR_EVT_SEND_PARAM_UPDATE;
                    *((uint16_t*) connList[i].pParamUpdateEventData->data) =
                            connHandle;

                    // Create a clock object and start
                    connList[i].pUpdateClock = (Clock_Struct*) ICall_malloc(
                            sizeof(Clock_Struct));

                    if (connList[i].pUpdateClock)
                    {
                        Util_constructClock(
                                connList[i].pUpdateClock,
                                multi_role_clockHandler,
                                SEND_PARAM_UPDATE_DELAY,
                                0, true,
                                (UArg) connList[i].pParamUpdateEventData);
                    }
                    else
                    {
                        // Clean up
                        ICall_free(connList[i].pParamUpdateEventData);
                    }
                }
                else
                {
                    // Memory allocation failed
                    MULTIROLE_ASSERT(false);
                }
            }
#endif

            break;
        }
    }

    return i;
}

/*********************************************************************
 * @fn      multi_role_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void multi_role_clearPendingParamUpdate(uint16_t connHandle)
{
    List_Elem *curr;

    for (curr = List_head(&paramUpdateList); curr != NULL;
            curr = List_next(curr))
    {
        if (((mrConnHandleEntry_t*) curr)->connHandle == connHandle)
        {
            List_remove(&paramUpdateList, curr);
        }
    }
}

/*********************************************************************
 * @fn      multi_role_removeConnInfo
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t multi_role_removeConnInfo(uint16_t connHandle)
{
    uint8_t connIndex = multi_role_getConnIndex(connHandle);

    if (connIndex < MAX_NUM_BLE_CONNS)
    {
        Clock_Struct *pUpdateClock = connList[connIndex].pUpdateClock;

        if (pUpdateClock != NULL)
        {
            // Stop and destruct the RTOS clock if it's still alive
            if (Util_isActive(pUpdateClock))
            {
                Util_stopClock(pUpdateClock);
            }

            // Destruct the clock object
            Clock_destruct(pUpdateClock);
            // Free clock struct
            ICall_free(pUpdateClock);
            // Free ParamUpdateEventData
            ICall_free(connList[connIndex].pParamUpdateEventData);
        }
        // Clear pending update requests from paramUpdateList
        multi_role_clearPendingParamUpdate(connHandle);
        // Clear Connection List Entry
        multi_role_clearConnListEntry(connHandle);
        numConn--;
    }

    return connIndex;
}
