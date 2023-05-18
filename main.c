#include <stdint.h>
#include <stdio.h>
#include <file.h>
#include <math.h>

#include "DSP28x_Project.h"     // DSP28x Headerfile
#include "sci_io.h"
#include "init.h"

#define TestPinOn()     GpioDataRegs.GPBSET.bit.GPIO33=1
#define TestPinOff()    GpioDataRegs.GPBCLEAR.bit.GPIO33=1
#define GetButton1()   (GpioDataRegs.GPBDAT.bit.GPIO55 ? 0:1)
#define GetButton2()   (GpioDataRegs.GPBDAT.bit.GPIO56 ? 0:1)
#define PI 3.14159265358979323846

extern void DSP28x_usDelay(Uint32 Count);
extern uint16_t uiSciMsgReceived;

Uint16 uiRxBuf[10]={0,0,0,0,0,0,0,0,0,0};
Uint16 uiSciLen=0;
Uin
uint16_t trimmerValue;
uint16_t jas=0;
uint16_t duty_cycle;
uint16_t delay_time;
uint16_t ticks = 0;
Uint8 stavB1;
Uint8 stavB2;

Uint8 iter=0;
Uint8 increment_direction=1;

//RMS vypocet
double value =0;
double root_mean_square =0;
double second_power_input = 0;
double total = 0;
double input_mapped =0;
uint16_t uiAnalogStatus =0;
uint64_t numberof_measurements =0;
//COUNTER
uint16_t delayStateMachineCounter=0;
uint16_t globalCounter;
uint16_t trimerDelay=0;
uint16_t waveDelay =0;
uint16_t readDelay =0;

//WAVE

Uint8 dac=1;
Uint8 up=0;
double dac_double = 0;
double period_ticks = 0;
double increment_triangle=0;
double sin_fix = 0;
//LEDS
typedef struct {
    uint16_t onTime;
    uint16_t timerOn;
    uint16_t timerOff;
    Uint8 ledState;
} LED_INFO;

// States StateMachine
#define INIT 1
#define START_UP 2
#define RUN 3
#define TEST1 4
#define TEST2 5
#define ERROR 6

Uint8 state = INIT;

/*blink led multiplayer from ms to micro s 300ms 300,000 micro s
 * but interrupt is 100 micro
*/
//States Other
#define TIMER_MULTIPLAYER 10
#define PERIOD 1000
#define STATE_RUN_DELAY 10000
#define BLUE 0
#define RED 1
#define SIN 11
#define TRIANGLE 22
LED_INFO * leds_global;



void SetLedBlue(uint16_t a)
{
    if(a)GpioDataRegs.GPBCLEAR.bit.GPIO39=1;
    else GpioDataRegs.GPBSET.bit.GPIO39=1;
}

void SetLedRed(uint16_t a)
{
    if(a)GpioDataRegs.GPBCLEAR.bit.GPIO34=1;
    else GpioDataRegs.GPBSET.bit.GPIO34=1;
}

// read value from ADC
int16_t GetADCINA6(void)
{
    AdcRegs.ADCSOCFRC1.bit.SOC1 = 1;
    return AdcResult.ADCRESULT1;
}

// read value from ADC
int16_t GetADCINA7(void)
{
    AdcRegs.ADCSOCFRC1.bit.SOC2 = 1;
    return AdcResult.ADCRESULT2;
}

// read value from ADC
int16_t GetADCINA4(void)
{
    AdcRegs.ADCSOCFRC1.bit.SOC14 = 1;
    return AdcResult.ADCRESULT14;
}

trimer(LED_INFO * led) {
    trimmerValue = GetADCINA6();
    jas = trimmerValue/16;
    if (trimmerValue < 50) {
        led->ledState = 1;
    } else if (trimmerValue > 3950) {
        led->ledState = 0;
    } else {
      duty_cycle = (jas * 100) / 255;
      delay_time = (duty_cycle * 255) / 10000;
      if(globalCounter - trimerDelay >= delay_time){
          led->ledState = 1;
                    trimerDelay = globalCounter;
    }else{
        led->ledState = 0;

    }

     /* DSP28x_usDelay(duty_cycle);

      DSP28x_usDelay(delay_time);*/
    }
}
//100us interrupt
__interrupt void cpu_timer0_isr(void)
{
    SciTmr();
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    globalCounter++;
}

setState(uint16_t stateArg){
    state = stateArg;
}

int blinkLed(LED_INFO * led, uint16_t ledOnTime, uint16_t period) {

    if (period ==0) {
        led->ledState = 0;
        return 0;
    }else if (ledOnTime >= period){
        led->ledState = 1;
        return 0;
    }
    led->onTime = ledOnTime;
    if (globalCounter - led->timerOn >= led->onTime * TIMER_MULTIPLAYER && led->ledState == 1) {
        led->timerOn = 0;
        led->ledState = 0;
        led->timerOff = globalCounter;
    }
    if (globalCounter - led->timerOff >= TIMER_MULTIPLAYER *(period -led->onTime) && led->ledState == 0) {
        led->timerOff = 0;
        led->ledState = 1;
        led->timerOn = globalCounter;
    }
    return 0;
}

LED_INFO * createLedStructs() {
    static LED_INFO leds[2];
    leds[BLUE].onTime = 0;
    leds[BLUE].timerOn = 0;
    leds[BLUE].timerOff = 0;
    leds[BLUE].ledState =0;

    leds[RED].onTime = 0;
    leds[RED].timerOn = 0;
    leds[RED].timerOff = 0;
    leds[RED].ledState =0;
    //moze byt definovane vo for cykle
    return leds;
}

StateMachine(LED_INFO *leds){

    stavB1= GetButton1();
    stavB2= GetButton2();
    switch (state) {
        case INIT:
            if(!stavB1 && !stavB2) {
                leds[BLUE].ledState = 0;
                leds[RED].ledState = 1;
                delayStateMachineCounter = 0;
            }

            if(stavB1 && !stavB2) {
                setState(START_UP);
                delayStateMachineCounter = globalCounter;
            }
        break;

        case START_UP:
            if(!stavB1 && !stavB2) {
                setState(INIT);
            }else {
                leds[BLUE].ledState = 1;
                leds[RED].ledState = 1;
                if(globalCounter - delayStateMachineCounter >= STATE_RUN_DELAY) {
                    setState(RUN);
                }
            }
        break;

        case RUN:
            leds[BLUE].ledState = 1;
            leds[RED].ledState = 0;
            if(!stavB1 && stavB2) {
                leds[BLUE].ledState = 1;
                setState(TEST1);
            }
            if(stavB1 && stavB2) {
                leds[BLUE].ledState = 1;
                setState(TEST2);
            }
        break;
        case TEST1:
            blinkLed(&leds[BLUE], 300, 1000);
            trimer(&leds[RED]);
            if (!stavB2) {
                setState(ERROR);
            }
        break;
        case TEST2:
            blinkLed(&leds[BLUE], 800, 1000);
            trimer(&leds[RED]);
            if (!stavB2) {
                setState(ERROR);
            }
        break;
        case ERROR:
            leds[BLUE].ledState = 0;
            blinkLed(&leds[RED], 500, 1000);
            if(!stavB1 && !stavB2){
                setState(INIT);
            }
        break;
        default:
        // code to execute if expression does not match any of the constants
          break;
    }
}

void setFrequency_Sinfix_ticks(Uint16 f) {
    period_ticks = (1/(double)f)*10000;
    increment_triangle = (double)255/(period_ticks/2);
    ticks = (int)period_ticks/2;
    sin_fix = 255/ period_ticks;
}

void makeTriangle() {
    if(globalCounter - waveDelay >=1 ){
        dac_double+=increment_triangle;
        up +=increment_direction;
            if (up <0 || up >= ticks) {
                increment_triangle *= -1;
                increment_direction *= -1;
            }
            EPwm7Regs.CMPA.half.CMPA = (Uint8)dac_double;
            waveDelay = globalCounter;
    }
}

void makeSine() {
    if(globalCounter - waveDelay >= 1){
                double sine = sin(2.0 * PI * iter / 256.0*sin_fix);
                dac = (Uint8) ((sine + 1.0) / 2.0 * 255.0);
                EPwm7Regs.CMPA.half.CMPA = dac;
                waveDelay = globalCounter;
                iter++;
    }
}
void makeWave(Uint8 value) {
    /*
     * pri isr preruseni na 100microSec treba na 1ms globalCounter= 10 tikov na 10ms
     * 100 tikov
     * vypocet pomocou funkcie je 1/f napr 1/50 je 0.02 co je 20ms
     *
     * 1 / (100 / 1000000)
     * */
    if(value){
       leds_global[BLUE].ledState = 1;
       makeSine();
    }else {
       leds_global[BLUE].ledState = 0;
       makeTriangle();
    }
}

void readValue(uint16_t input,double volt_logic) {
    input_mapped = ((double)input * (volt_logic/4096));
    if(globalCounter - readDelay >= 10){
        numberof_measurements++;
        second_power_input = pow(input_mapped,2);
        total += second_power_input;
        root_mean_square = sqrt((1/(double)numberof_measurements)*total);
    }
    if(GetButton2()) {
        numberof_measurements = 0;
        total = 0;
    }
}
void sendRequest() {
    /*
     *
    ÑCmd1ì Poölite nameranÈ (prÌp.1,2V) nap‰tie v 1mV (LSB First) v sekcii DATA
    ÑCmd2ì Poölite Vaöe krstnÈ meno (v ASCII znakoch) v sekcii DATA
    ÑCmd3ì Poölite n·zov firmy BEL ÑCmd4ì v sekcii DATA
    ÑCmd4ì Povoliù posielanie hodnoty nap‰tia v 1s intervale z ˙lohy Ë.4 (prÌp.230V)
    ÑCmd5ì Zak·zaù posielanie hodnoty nap‰tia v 1s intervale z ˙lohy Ë.4 (prÌp.230V)
    Byte V˝znam a obsah bajtu
    1 0xAA = Start byte
    2 DÂûka celej spr·vy vr·tane CRC a start bajtu
    3 0x1B = Adresa odosielateæa
    4 ID prÌkazu
    5...n-1 DATA
    n CRC = suma bajtov 1 aû (n-1)
     */
    //moj navrh
   //1 uint16_t startByte = 0xAA;
    //2 uint16_t  v com sa lisi od CRC?je to teda suma bytov 1(start byte) a n(crc) + Data suma?
    //teda startbyte (2Bytes) + CRC (2Bytes) + DATA (n*2BYTES)
    //3 uint16_t sender = 0x1B pre jedneho 0x1C pre druheho (cize jedno pre odosielatela a jedno pre prijimatela(neskor odosielatela)
    //4 uint16_t id_request = napr pre CMD4 to bude 4. To znamena 1.user posle id= 4 2. user chyti,spracuje a posiela naspat tiez id=4
    //5 pri Data CMD1 ulozim napriklad hodnotu 1.98V na 1980mV a posledny bit prvy cize 0000 0111 1011 1100 (1980) invertovat na 0011 1101 1110 0000
    SciWriteByte(uiR)
}
void processMessage() {

}

void main(void)
{
    /* SetLedRed(1);
    // SetLedBlue(1);
    // uint16_t uiButton1Status = GetButton1();
    // uint16_t uiButton2Status = GetButton2();
    // uint16_t uiRegIn = GetADCINA4();
    // uint16_t uiRegDesire = GetADCINA6();
    // uint16_t uiTrimerStatus = GetADCINA6(); // 12bit
    //  // 12bit
    // double a = sin(double);
    // EPwm7Regs.CMPA.half.CMPA = 0; //signal generator
    // EPwm8Regs.CMPB //regulator output
    // SciWriteByte(uint16_t byte);
    // putchar(char);
    // printf(
    */
    Init();
    leds_global = createLedStructs();
    waveDelay = globalCounter;
    readDelay = globalCounter;
    iter = 0;
    EPwm7Regs.CMPA.half.CMPA = 0;
    setFrequency_Sinfix_ticks(70);
    while(1) {
       makeWave(GetButton1());
       readValue(GetADCINA7(),3.3);            //StateMachine(leds_global);
       uiAnalogStatus = GetADCINA7();
       SetLedRed(leds_global[RED].ledState);
       SetLedBlue(leds_global[BLUE].ledState);
       //blinkLed(&leds_global[BLUE], 300, 1000);
       //trimer(&leds_global[BLUE]);
    }

    for(;;)
    {

        if(uiSciMsgReceived)//some message from SCI is received
        {
            uiSciLen = SciReadMsg(uiRxBuf);//read buff and buffer length
        }
    }
}
