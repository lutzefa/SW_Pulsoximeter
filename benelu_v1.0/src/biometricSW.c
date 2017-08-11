/*******************************************************************************
    Project:                Pulsoximeter
    Program name:           biometricSW.c
    Author:                 Lutz Willert
    Date created:           08.08.2017
    Version:                1.0
    Date version created:   08.08.2017
    Revision history:       Added comments and deleted everything not
                            used/needed we know by now.
 ******************************************************************************/

/*******************************************************************************
 *  Includes - not touched, right out of the demo
 ******************************************************************************/

#include "em_device.h" 					//LW: Define device & include header --> EFM32WG990F256
										//LW: CMSIS Cortex-M Peripheral Access Layer for Silicon Laboratories microcontroller devices
#include "em_chip.h"					//LW: Chip initialization API & chip initialization routine for revision errata workarounds
#include "em_cmu.h"						//LW: Clock management unit (CMU) API
#include "em_emu.h"						//LW: Energy management unit (EMU) peripheral API
#include "em_gpio.h"					//LW: General purpose IO (GPIO) peripheral API, GPIO pin modes, enable/disable serial wire clock pin, ...
#include "i2cspm.h"						//LW: I2C simple poll-based master mode driver for the DK/STK, I2C driver instance initialization structure
/*TODO: si7013.h koennte geloescht werden, wenn nicht benoetigt */
#include "si7013.h"						//LW: Driver for the Si7013 temperature / humidity sensor
#include "heart_rate_monitor.h"			//LW: Heart Rate and SpO2 state machine
#include "rtcdriver.h"					//LW: RTCDRV timer API definition --> Real time clock driver
#include "graphics.h"					//LW: Draws the graphics on the display
#include "em_adc.h"						//LW: Analog to Digital Converter (ADC) peripheral API
#include "udelay.h"						//LW: Microsecond delay routine
#include "si114x_functions.h"			//LW: Si114x function prototypes, structure and bit definitions
#include "si114x_uv_index.h"			//LW: Si114x UV Index measurement function prototypes
#include "usb_debug.h"					//LW: USB device setup for HRM debugging
#include "si114x_sys_out.h"				//LW: Implementation specific functions for HRM code
#include "si114x_functions.h"			//LW: Si114x function prototypes, structure and bit definitions
#include "si114xhrm_user_functions.h"	//LW: Si114x functions to be provided by developer specific to the system in which the HRM algorithm is used
#include "bsp.h"						//LW: Board support package API definitions

#define BIOMETRIC_DEMO_VERSION "LW 1.1"	//LW: What's written in " " is displayed while startup and shows version number, i.e. in this case "LW 1.1"

/*******************************************************************************
*   Meine eigenen defines, um bestimmte Funktionen ab- und zuzuschalten
 ******************************************************************************/

/** LW: si7013 zuschalten. inklusive I2C_2
    -   -> zum Abschalten auskommentieren
*/
#define LW_si7013_ENABLE
/** LW: STK LEDs, die beiden kleinen unterhalb vom Display zuschalten.
        --> zum Abschalten auskommentieren
*/
#define LW_STK_LEDs_ENABLE
/** LW: Display zuschalten.
        --> zum Abschalten auskommentieren
*/
#define LW_DISPLAY_ENABLE

/*******************************************************************************
*   Local defines
 ******************************************************************************/

/** Time (in ms) between periodic updates of the measurements.
        LW: Hier wird jetzt 1/min (60000) geprueft, ob SkinContact vorhanden ist.
            Im Original Demo 1/sek (1000). Zeitspanne erhoehen, um Energie zu sparen,
            bzw. Zeit zw. Messungen festzulegen.
*/
#define PERIODIC_UPDATE_MS      60000
/** Time (in ms) between characters of the scrolling text.
        LW:	Voreingestellt in der Demo waren 250
*/
#define UPDATE_SCROLL_MS        175
/** Time (in ms) between starts of the scrolling text.
        LW:	Voreingestellt in der Demo waren 6000
*/
#define START_SCROLL_MS         2000
/** Voltage defined to indicate dead battery. */
#define LOW_BATTERY_THRESHOLD   2900

/* I2C port/pins/addr */
#define SI114X_I2C_SCL_PORT     gpioPortD
#define SI114X_I2C_SCL_PIN      7
#define SI114X_I2C_SDA_PORT     gpioPortD
#define SI114X_I2C_SDA_PIN      6
#define SI114X_I2C_PORT_LOC     1
#define SI114X_NON_UV           0x5A
#define SI114X_UV               0x60

/*TODO: Kann eigentlich geloescht werden, hier ueber ifdef geloest, um
ein- und auszuschalten */
#ifdef  LW_si7013_ENABLE
    #define SI7013_I2C_SCL_PORT     gpioPortC
    #define SI7013_I2C_SCL_PIN      5
    #define SI7013_I2C_SDA_PORT     gpioPortC
    #define SI7013_I2C_SDA_PIN      4
    #define SI7013_I2C_PORT_LOC     0
    #define SI7013_ADDR_0           (0x41 << 1)
    #define SI7013_ADDR_1           (0x40 << 1)
#endif

/* LED and IRQ ports/ pins */
#define SI114X_INT_PORT         gpioPortD
#define SI114X_INT_PIN          5
#define RED_LED_PORT            gpioPortC
#define RED_LED_PIN             3
#define GREEN_LED_PORT          gpioPortC
#define GREEN_LED_PIN           0
#define PB0_PORT                gpioPortB
#define PB0_PIN                 9
#define PB1_PORT                gpioPortB
#define PB1_PIN                 10

/*TODO:ifdef eingefuegt, um die STK LEDs unter dem Display
ein- und auszuschalten */
#ifdef LW_STK_LEDs_ENABLE
    #define LED0_PORT               gpioPortE
    #define LED0_PIN                2
    #define LED1_PORT               gpioPortE
    #define LED1_PIN                3
#endif

/*******************************************************************************
 * Local variables
 ******************************************************************************/

/** This flag tracks if we need to update the display (measurements). */
static volatile bool updateDisplay = true;
/** This flag tracks if we need to scroll a new character. */
static volatile bool updateScrollText = false;
/** This flag tracks if we need to perform a new measurement. */
static volatile bool updateMeasurement = true;
/** Flag used to indicate ADC is finished */
static volatile bool adcConversionComplete = false;     //TODO: rausfinden, welcher Analogwert
/** Flag used to indicate lcd boost enabled */
static bool oldBoost = false;
/** Flag used to indicate what to diplay */
static volatile displayType_t displayType = LCD_HRM;    //TODO: Teile loeschen
/** Flag used to indicate whether HRM is active. */
static bool hrmActive = false;
/** HRM hardware configuration */
static HeartRateMonitor_Config_t hrmConfig = BIOMETRIC_EXP;
/** Flag used to restart HRM algorithm */
static bool reinitHRM = false;
/** Flag used to determine if USB is enabled */
static bool usbEnabled = false;

/** Timer used for periodic update of the measurements. */
RTCDRV_TimerID_t periodicUpdateTimerId;
/** Timer used for scrolling text. */
RTCDRV_TimerID_t updateScrollTimerId;

/** Driver handle instances */
/*TODO: ifdef LW_si7013_ENABLE um ein und auszuschalten*/
#ifdef LW_si7013_ENABLE
    I2CSPM_Init_TypeDef si7013I2CHandle = I2CSPM_INIT_DEFAULT;
#endif
I2CSPM_Init_TypeDef si114xI2CHandle = I2CSPM_INIT_DEFAULT;
Si114xPortConfig_t* si114xDrvHandle;
Si114xPortConfig_t uvPort;
Si114xPortConfig_t hrmPort;


/*******************************************************************************
 * Local prototypes
 ******************************************************************************/
static void gpioSetup(void);
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user);
static void updateScrollTextCallback(RTCDRV_TimerID_t id, void *user);
static uint32_t checkBattery(void);
static void adcInit(void);
static void configSi114xIntInput (void);

/*******************************************************************************
 * @brief  Helper function to perform data measurements.
 ******************************************************************************/
static int32_t performMeasurements(uint32_t *rhData, int32_t *tData,
                                   uint16_t *uvData, uint32_t *vBat)
{
  bool vboost;
  *rhData = 1234;   //TODO: wird eigentlich nicht benoetigt
  *tData = 5678;    //TODO: wird eigentlich nicht benoetigt
  *vBat = checkBattery();

//TODO: ifdef LW_si7013_ENABLE, um ein und auszuschalten */
#ifdef LW_si7013_ENABLE
    Si7013_MeasureRHAndTemp(si7013I2CHandle.port, SI7013_ADDR_0, rhData, tData);
#endif

  Si114x_MeasureUVIndex(uvData); /*TODO: pruefen, ob benoetigt */

  vboost = (*vBat < 2900);

  if (vboost != oldBoost)
  {
    GRAPHICS_Init(vboost);
    oldBoost = vboost;
  }
  return 0;
}

/*******************************************************************************
 * @brief  Detects HRM device on ribbon cable.
 ******************************************************************************/
/*TODO: hiervon kann viel weg, wenn kein check fuer die anderen Sensoren
So koennte der Block umgeschrieben dann aussehen und ist funktionsfaehig, wenn
auch noch nicht ganz verstanden.
Der Block darunter ist das Original komplett getoggled*/
static void detectHRMDevice(void){
	hrmConfig = BIOMETRIC_EXP;													//LW: hier BIOMETRIC_EXP folgen, da kann was gel�scht werden
	hrmPort.i2cAddress = SI114X_UV;
		if (Si114xInit(&hrmPort, 0, (HANDLE)&si114xDrvHandle) < 0)
		{
			hrmPort.i2cPort = I2C0;
		}
}

// static void detectHRMDevice(void)
// {
//   hrmConfig = BIOMETRIC_EXP;
//   /* Check for Si1143 or other device on i2c1 6 pin flex cable connector
//      If we detect another EVB connected via flex cable switch heart rate
//      configuration to use the EVB connected instead of the Si1146 on the
//      EXP board. */
//
//   /* First check for Si1143 on address 0x5A on flex cable. */
//   hrmPort.i2cAddress = SI114X_NON_UV;
//   hrmPort.i2cPort = I2C1;
//   if (Si114xInit(&hrmPort, 0, (HANDLE)&si114xDrvHandle) < 0)
//   {
//     /* We didn't find an Si1143.
//        If no Si1143 detected, check for the Si1147 postage stamp */
//     hrmPort.i2cAddress = SI114X_UV;
//     if (Si114xInit(&hrmPort, 0, (HANDLE)&si114xDrvHandle) < 0)
//     {
//       hrmPort.i2cPort = I2C0;
//     }
//     else
//     {
//       /* Found a Si1147 so reconfigure */
//       hrmConfig = SI1147_PS;
//     }
//   }
//   else
//   {
//     /* Found an Si1143. Need to discover which EVB and reconfigure.
//        Set up HRM i2c parameters for Si1143 on i2c bus 1.
//        Figure out which board we are connected to.
//        If we detect an Si7013 also on i2c1 then we have a fitness EVB. */
//     if (Si7013_Detect(si7013I2CHandle.port, SI7013_ADDR_1, NULL))
//     {
//       /* Fitness evb detected (wrist). - we do not support wrist based
//          operation on the biometric EXP. Contact Silicon Labs for a wrist
//          based HRM solution. */
//       GRAPHICS_DrawError();
//       while(1)
//       {
//         if ((USBD_SafeToEnterEM2() && usbEnabled) || (!usbEnabled))
//         {
//           EMU_EnterEM2(usbEnabled);
//         }
//       }
//     }
//     else
//     {
//       /* If we did not find a Si7013 then it must be a Si1143 postage stamp. */
//       hrmConfig = SI1143_PS;
//     }
//   }
// }

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  uint32_t         rhData;          /*TODO: Koennte wahrscheinlich geloescht werden */

/*TODO: ifdef LW_si7013_ENABLE, um ein und auszuschalten */
#ifdef LW_si7013_ENABLE
  bool             si7013Status;
#endif

  bool             si1146Status;    //Dieser Sensor ist auf unserem EXP Board
  int32_t          tempData;        /*TODO: Koennte wahrscheinlich geloescht werden */
  uint16_t         uvData = 50;     /*TODO: Koennte evtl. geloescht werden */
  uint32_t         vBat = 3300;
  bool             lowBatPrevious = true;
  bool             lowBat = false;

  char             hrmVersion[10];  /*TODO: Es koennte die eigene Version
  eingetragen werden. Dafuer hier evtl. Groeße des arrays aendern */
  bool             scrollStatus;
  bool             hrmCheckSkinContact = true;

  /* si114x I2C driver config */
  si114xI2CHandle.port = I2C0;
  si114xI2CHandle.sclPort = SI114X_I2C_SCL_PORT;
  si114xI2CHandle.sclPin = SI114X_I2C_SCL_PIN;
  si114xI2CHandle.sdaPort = SI114X_I2C_SDA_PORT;
  si114xI2CHandle.sdaPin = SI114X_I2C_SDA_PIN;
  si114xI2CHandle.portLocation = SI114X_I2C_PORT_LOC;

/*TODO: ifdef LW_si7013_ENABLE, um ein und auszuschalten*/
#ifdef LW_si7013_ENABLE
  /* si7013 I2C driver config */
  si7013I2CHandle.port = I2C1;
  si7013I2CHandle.sclPort = SI7013_I2C_SCL_PORT;
  si7013I2CHandle.sclPin = SI7013_I2C_SCL_PIN;
  si7013I2CHandle.sdaPort = SI7013_I2C_SDA_PORT;
  si7013I2CHandle.sdaPin = SI7013_I2C_SDA_PIN;
  si7013I2CHandle.portLocation = SI7013_I2C_PORT_LOC;
#endif

  /* si114x UV/HRM port config */
  uvPort.i2cPort = si114xI2CHandle.port;
  uvPort.i2cAddress = SI114X_UV;
  uvPort.irqPin = SI114X_INT_PIN;
  uvPort.irqPort = SI114X_INT_PORT;

  hrmPort.i2cPort = si114xI2CHandle.port;
  hrmPort.i2cAddress = SI114X_NON_UV;
  hrmPort.irqPin = SI114X_INT_PIN;
  hrmPort.irqPort = SI114X_INT_PORT;

  updateMeasurement = false;

  /* Chip errata */
  CHIP_Init();

  /* Misc initalizations. */
  gpioSetup();
  adcInit();

  /* Misc timers. */
  RTCDRV_Init();

  RTCDRV_AllocateTimer(&periodicUpdateTimerId);
  RTCDRV_AllocateTimer(&updateScrollTimerId);

  /* Initialize display. */
/*TODO: ifdef eingefuegt, um das Display ein- und ausschalten zu koennen */
  #ifdef LW_DISPLAY_ENABLE
        GRAPHICS_Init(false);
  #endif

/*TODO: USB fuer Datentransfer der "Rohdaten". Pruefen ob if-struktur so richtig
oder ob else zweig und if!usbEnabled anders sein sollte. sieht komisch aus */
  /* Check if we need to enable USB.
     USB is enabled by holding down PB0 during reset */
  if (!GPIO_PinInGet(PB0_PORT, PB0_PIN)) /* PB0 is pressed */
  {
    /* USB initialization */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    CMU_OscillatorEnable(cmuOsc_LFXO, true, false);
    USBDebug_Init();
    usbEnabled = true;
  }
  CMU_OscillatorEnable(cmuOsc_HFXO, true, true);
  /* Enable HFXO as the main clock */
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );

  /* Save power if we are not using USB peripheral */
  if (!usbEnabled)
  {
    CMU_ClockDivSet(cmuClock_HF,cmuClkDiv_8);
  }

//TODO: Pruefen,ob benoetigt
#if !defined( BSP_STK )
  BSP_PeripheralAccess(BSP_I2C, true);
#endif

  /* Initialize I2C drivers for si114x and si7013 using standard rate */
   I2CSPM_Init(&si114xI2CHandle);

/* TODO: #ifdef LW_si7013_ENABLE, um ein und auszuschalten */
#ifdef LW_si7013_ENABLE
   I2CSPM_Init(&si7013I2CHandle);
  /* Detect Si7013 device on EXP board */
  si7013Status = Si7013_Detect(si7013I2CHandle.port, SI7013_ADDR_0, NULL);
#endif

  /* Detect Si1146 device on EXP board. */
  si1146Status = (Si114xInit(&uvPort, 0, (HANDLE)&si114xDrvHandle) >= 0);

//TODO: Koenntenweggelassen/veraendert werden, wenn device eindeutig
  detectHRMDevice();

  HeartRateMonitor_GetVersion(hrmVersion);

  /* Show splash screens informing user of configuration. */
/*TODO: anpassen, welche Daten angezeigt werden sollen */
  GRAPHICS_DrawInit(hrmConfig, hrmVersion,BIOMETRIC_DEMO_VERSION, usbEnabled);

  /* If EXP board fails display error message. */
/* TODO: si7013 status Koennte geloescht werden, hier getoogled */
  if ((!si1146Status) /*|| (!si7013Status)*/)
  {
    GRAPHICS_DrawError();
    if ((USBD_SafeToEnterEM2() && usbEnabled) || (!usbEnabled))
    {
      EMU_EnterEM2(usbEnabled);
    }
    while(true);
  }

  RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                    PERIODIC_UPDATE_MS, periodicUpdateCallback, NULL);

  updateDisplay = true;

  HeartRateMonitor_Init(&hrmPort, hrmConfig);

  /* Switch Si114x driver to talk to Si1146 on EXP board to initialize UV measurement. */
  if (hrmConfig != BIOMETRIC_EXP)
  {
    Si114xInit(&uvPort, 0, (HANDLE)&si114xDrvHandle);
  }
  Si114x_ConfigureUV(hrmConfig != BIOMETRIC_EXP, (HANDLE)&si114xDrvHandle);

//TODO: hier koennten die nicht benoetigten messungen entfallen, pruefen,wie das gemacht werden kann
  performMeasurements(&rhData, &tempData, &uvData, &vBat);

  /* Configure Si114x interrupt gpio input. */
  configSi114xIntInput();
  reinitHRM = true;


/*TODO: hier beginnt der LOOP --> pruefen, was weggelassen werden kann, wenn
wir nur HRM & SPO2 machen. zum Beispiel display modes naechster Absatz*/
  while (true)
  {
    /* There are two display mode. HRM-SpO2 and RH/T/UV modes.
       HRM-SpO2 requires an accurate clock and so cannot use EM2
       when HRM-SpO2 measurement is active. The HRM-SpO2 state
       machine is implemented in heartratemonitor.c. */
    if ((displayType == LCD_HRM) || (displayType == LCD_SPO2))
    {
      if (reinitHRM)
      {
        /* If HRM-SpO2 was not previously displayed (ie we have changed
           modes then perform some initialization. */
    	if (hrmConfig == BIOMETRIC_EXP)
        {
    	  Si114x_EnableVisRange(0);  /* HRM requires VIS_RANGE = 0 */
        }

        HeartRateMonitor_Loop(false, true, false); /* Shut down HRM */
        reinitHRM = false;
        Si114xInit(&hrmPort, 0, (HANDLE)&si114xDrvHandle);
        if (displayType == LCD_HRM)
        {
          GRAPHICS_ShowHRMStatus(false, 0, false);
        }
        else
        {
          GRAPHICS_ShowSpO2Status(false, 0, false);
        }
        hrmCheckSkinContact = true;
        RTCDRV_StopTimer(updateScrollTimerId);
        RTCDRV_StartTimer(updateScrollTimerId, rtcdrvTimerTypePeriodic,
                                          START_SCROLL_MS, updateScrollTextCallback, NULL);
      } /* if (reinitHRM) */
      if (updateMeasurement)
      {
        updateMeasurement = false;
        hrmCheckSkinContact = true;
        updateDisplay = false;
      }
      /* Enter HRM loop.  */
      hrmActive = HeartRateMonitor_Loop(displayType == LCD_SPO2, false, hrmCheckSkinContact);
      hrmCheckSkinContact = false;
      if (!hrmActive)
      {
        /* we are not actively processing samples so we just periodically
         * check for skin contact.
         */
        if (updateScrollText)
        {
          /* scroll instructions to user across LCD*/

/*TODO: ifdef LW_DISPLAY_ENABLE, um ein und auszuschalten */
#ifdef LW_DISPLAY_ENABLE
          updateScrollText = false;
          if (displayType == LCD_HRM)
          {
            scrollStatus =  GRAPHICS_ShowHRMStatus(false, 0, true);
          }
          else
          {
            scrollStatus =  GRAPHICS_ShowSpO2Status(false, 0, true);
          }
          if (!scrollStatus)
          {
            RTCDRV_StartTimer(updateScrollTimerId, rtcdrvTimerTypePeriodic,
                            UPDATE_SCROLL_MS, updateScrollTextCallback, NULL);
          }
          else
          {
            RTCDRV_StartTimer(updateScrollTimerId, rtcdrvTimerTypePeriodic,
                                  START_SCROLL_MS, updateScrollTextCallback, NULL);
            if (displayType == LCD_HRM)
            {
              GRAPHICS_ShowHRMStatus(false, 0, false);
            }
            else
            {
              GRAPHICS_ShowSpO2Status(false, 0, false);
            }
          }
#endif
        }
        if ((USBD_SafeToEnterEM2() && usbEnabled) || (!usbEnabled))
        {
          EMU_EnterEM2(true); /* Must restore clocks for HRM */
        }
        else
        {
          EMU_EnterEM1();
        }
      } /* if (!hrmActive) */
      else
      {
        if (!HeartRateMonitor_SamplesPending())
        {
          EMU_EnterEM1();
        }
      } /* else (hrmActive) */
    }
    else
    {
      if((!reinitHRM) && (hrmConfig == BIOMETRIC_EXP))
      {
	/* Did we just enter UV/RH/T mode */
        Si114x_EnableVisRange(1);   /* UV requires VIS_RANGE = 1 */
      }
      reinitHRM = true;
      /* Shut down HRM */
      HeartRateMonitor_Loop(false, true, false);
      /* Initialize UV mode */
      Si114xInit(&uvPort, 0, (HANDLE)&si114xDrvHandle);

      if (updateMeasurement)
      {
/*TODO: pruefen, was hier wegfallen kann und wie: rh, temp*/
        performMeasurements(&rhData, &tempData, &uvData, &vBat);
        updateMeasurement = false;
        if (lowBatPrevious)
        {
          lowBat = (vBat <= LOW_BATTERY_THRESHOLD);
        }
        else
        {
          lowBat = false;
        }
        lowBatPrevious = (vBat <= LOW_BATTERY_THRESHOLD);
/*TODO: ifdef LW_DISPLAY_ENABLE, um ein und auszuschalten */
#ifdef LW_DISPLAY_ENABLE
        GRAPHICS_SetBatteryStatus(lowBat);
#endif
      }

/*TODO: ifdef LW_DISPLAY_ENABLE, um ein und auszuschalten */
#ifdef LW_DISPLAY_ENABLE
      if (updateDisplay)
      {
        switch (displayType)
        {
          case LCD_TEMPC:
            GRAPHICS_DrawTemperatureC(tempData);
            break;

          case LCD_TEMPF:
            GRAPHICS_DrawTemperatureF(tempData);
            break;

          case LCD_UV:
            GRAPHICS_DrawUV(uvData);
            break;

          case LCD_RH:
            GRAPHICS_DrawHumidity(rhData);
            break;

          default:
            GRAPHICS_DrawError ();
        }
        updateDisplay = false;
        /* Reset timer for periodic update of the display */
        RTCDRV_StartTimer(periodicUpdateTimerId, rtcdrvTimerTypePeriodic,
                          PERIODIC_UPDATE_MS, periodicUpdateCallback, NULL);

      }
#endif


      if ((USBD_SafeToEnterEM2() && usbEnabled) || (!usbEnabled))
      {
        EMU_EnterEM2(true);
      }
    }
  }
}


/**************************************************************************//**
 * @brief This function is called whenever we want to measure the supply v.
 *        It is reponsible for starting the ADC and reading the result.
 *****************************************************************************/
static uint32_t checkBattery(void)
{
  uint32_t vData;
  /* Sample ADC */
  adcConversionComplete = false;
  ADC_Start(ADC0, adcStartSingle);
  while (!adcConversionComplete)
  {
    EMU_EnterEM1();
  }
  vData = ADC_DataSingleGet(ADC0);
  return vData;
}


/**************************************************************************//**
 * @brief ADC Interrupt handler (ADC0)
 *****************************************************************************/
void ADC0_IRQHandler( void )
{
   uint32_t flags;

   /* Clear interrupt flags */
   flags = ADC_IntGet(ADC0);
   ADC_IntClear(ADC0, flags);

   adcConversionComplete = true;
}


/**************************************************************************//**
 * @brief ADC Initialization
 *****************************************************************************/
static void adcInit(void)
{
   ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
   ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

   /* Enable ADC clock */
   CMU_ClockEnable( cmuClock_ADC0, true );

   /* Initiate ADC peripheral */
   ADC_Init(ADC0, &init);

   /* Setup single conversions for internal VDD/3 */
   initSingle.acqTime = adcAcqTime16;
   initSingle.input   = adcSingleInpVDDDiv3;
   ADC_InitSingle(ADC0, &initSingle);

   /* Manually set some calibration values */
   ADC0->CAL = (0x7C << _ADC_CAL_SINGLEOFFSET_SHIFT) | (0x1F << _ADC_CAL_SINGLEGAIN_SHIFT);

   /* Enable interrupt on completed conversion */
   ADC_IntEnable(ADC0, ADC_IEN_SINGLE);
   NVIC_ClearPendingIRQ( ADC0_IRQn );
   NVIC_EnableIRQ(ADC0_IRQn);
}


/**************************************************************************//**
 * @brief  Set up GPIO input for Si114x interrupt line.
 *****************************************************************************/
static void configSi114xIntInput(void)
{
  /* Configure PD5 as input and enable interrupt - si114x interrupt. */
  /* Interrupt is active low */
  GPIO_PinModeSet(SI114X_INT_PORT, SI114X_INT_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(SI114X_INT_PORT, SI114X_INT_PIN, false, true, true);
}


/**************************************************************************//**
 * @brief Setup GPIO interrupt for pushbuttons.
 *****************************************************************************/
static void gpioSetup(void)
{
  /* Enable GPIO clock. */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PB9 as input and enable interrupt. (PB0) */
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, true, true);

  /* Configure PB10 as input and enable interrupt. (PB1) */
  GPIO_PinModeSet(PB1_PORT, PB1_PIN, gpioModeInputPull, 1);
  GPIO_IntConfig(PB1_PORT, PB1_PIN, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Configure PD4 as pushpull. (5v signal) */
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);
  GPIO_PinOutClear(gpioPortD, 4);

  /* Configure PC3 as pushpull. (Green LED) */
  GPIO_PinModeSet(GREEN_LED_PORT, GREEN_LED_PIN, gpioModeWiredAndPullUp, 0);
  GPIO_PinOutSet(GREEN_LED_PORT, GREEN_LED_PIN);

  /* Configure PC0 as pushpull. (Red LED) */
  GPIO_PinModeSet(RED_LED_PORT, RED_LED_PIN, gpioModeWiredAndPullUp, 0);
  GPIO_PinOutSet(RED_LED_PORT, RED_LED_PIN);

  /*si114x interrupt. we want this pin high while si114x starts up*/
  GPIO_PinModeSet(SI114X_INT_PORT, SI114X_INT_PIN, gpioModePushPull, 1);

/*TODO: ifdef LW_STK_LEDs_ENABLE, um diese auszuschalten, evtl mode aendern, damit ganz aus*/
#ifdef LW_STK_LEDs_ENABLE
  /* Configure PE2 as pushpull. (STK LED) */
  GPIO_PinModeSet(LED0_PORT, LED0_PIN, gpioModePushPull, 0);
  GPIO_PinOutClear(LED0_PORT, LED0_PIN);

  /* Configure PE3 as pushpull. (STK LED) */
  GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, 0);
  GPIO_PinOutClear(LED1_PORT, LED1_PIN);
#endif
}


/**************************************************************************//**
 * @brief Performs action when PB0/1 is pressed.
 *****************************************************************************/
/*TODO: pruefen, was hier wegfallen kann und wie am besten umzusetzen*/

static void PBPressed(int32_t button)
{
  /* Push button 0 changes display in negative direction. */
  /* Push button 1 changes display in positive direction. */
  if (button == 1)
  {
    if (displayType == LCD_UV)
    {
      displayType = LCD_HRM;
    }
    else
    {
      displayType++;
    }
  }
  else
  {
    if (displayType == LCD_HRM)
    {
      displayType = LCD_UV;
    }
    else
    {
      displayType--;
    }
  }
 /* We do not display SPO2 for all cases */
  if ((displayType == LCD_SPO2) && (hrmConfig != BIOMETRIC_EXP))
  {
    displayType = (displayType_t)((int)LCD_SPO2 + 1);
  }
  if ((displayType == LCD_SPO2) || (displayType == LCD_HRM))
  {
    reinitHRM = true;
  }
  updateDisplay = true;
}


/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB1)
 *****************************************************************************/
/*TODO: pruefen, ob nicht PB1 wegfallen kann. schaltet display weiter auf andere werttypen wie auch PB0 */
void GPIO_EVEN_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << PB1_PIN);
  PBPressed(1);
}


/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB0,Si1147 INT)
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  uint32_t flags;
  flags = GPIO_IntGet();
  if (flags & (1 << PB0_PIN))
  {
    PBPressed(0);
  }
  if (flags & (1 << SI114X_INT_PIN))
  {
    /* Si114x IRQ line */
    HeartRateMonitor_Interrupt();
  }
  GPIO_IntClear(flags);
}


/**************************************************************************//**
 * @brief Callback used to count between measurement updates
 *****************************************************************************/
static void periodicUpdateCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  updateDisplay = true;
  updateMeasurement = true;
}


/**************************************************************************//**
 * @brief Callback used to count between scrolling updates
 *****************************************************************************/
static void updateScrollTextCallback(RTCDRV_TimerID_t id, void *user)
{
  (void) id;
  (void) user;
  updateScrollText = true;
}


/************************************************************
 *      @brief  SYSTICK interrupt handler
 ************************************************************/
void SysTick_Handler(void)
{
  HeartRateMonitor_TimerEventHandler();
}
