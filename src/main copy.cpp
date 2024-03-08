#include <Arduino.h>
#include <GyverFIFO.h>
#include <TimerMs.h>
#include <OneButton.h>
////////////////Configuration////////////////
// #define DEBUG

/////////////////////
// Declaring Delay`s
#define TMR_PERIOD 20  // Period of timer (20ms., 50Hz.)
#define SCAN_PERIOD 10 // Delay to scan in millisec. (10ms., 100Hz.)
#define IND_PERIOD 40  // Delay to update indication (40ms., 25Hz.)
// #define TIMEOUT 10000  // Timeout for direct transmit message (8000ms., 8s.)

// static byte wdg_state = 0x00;

//////////////////////
// Описание контактов
// Набор контактов для подключения переключателя задержки таймера RS-10
#define P1 0x02
#define P2 0x03
#define P4 0x04
#define P8 0x05

#define PINBTN 0x06 // Подключена кнопка       | Input
#define RLY 0x07    // реле эмуляции кнопки    | Output

#define SERIAL_TX_BUFFER_SIZE 1024
#define SERIAL_RX_BUFFER_SIZE 1024

namespace flprog
{
    uint32_t difference32(uint32_t start, uint32_t end)
    {
        if (end >= start)
        {
            return end - start;
        }
        return (0xfFFFFFFF - start) + end;
    }
    bool isTimer(uint32_t startTime, uint32_t period)
    {
        return (difference32(startTime, (millis()))) >= period;
    }
    bool isTimerMicros(unsigned long startTime, unsigned long period)
    {
        return (difference32(startTime, (micros()))) >= period;
    }
}

OneButton button(PINBTN); // Подключение кнопки
GyverFIFO<byte, 512> buf;

TimerMs timeoutTmr(1000, 0, 1);
TimerMs mainTmr(TMR_PERIOD);

const byte start_msg = 0x02; // (02) Символ начала посылки
const byte end_msg = 0x03;   // (03) Символ конца посылки

static int timeoutValue = 0;
static volatile bool directSend = true;
static volatile bool msg_received = false;
static volatile bool msg_transmitted = false;

static volatile bool relayEnable = 0;
static volatile bool t_on_enable = 0;
static volatile bool t_on_out = 0;
static volatile unsigned long t_on_time = 0UL;
static volatile bool one_gen_enable = 0;
static volatile bool one_gen_out = 0;
static volatile unsigned long one_gen_time = 0UL;
static volatile bool FTrig_1_Out = 0;
static volatile bool FTrig_1_OldStat = 0;

static bool button_enabled = true;

static volatile bool relayClick = false;

void receive();
void transmit();
void calcDelay();
void myDelay();
// void relayClick();
void buttonClick();

void receive()
{
    if (!Serial.available())
        return;
    mainTmr.stop();
    msg_received = false;

    byte val = 0x00;
    while (Serial.available())
    {
        val = (byte)Serial.read();
        if (val == start_msg)
        {
            buf.clear();
        }
        buf.write(val);
    }
    if (val == end_msg)
    {
        msg_received = true;
        msg_transmitted = false;
    }
    mainTmr.restart();
    timeoutTmr.restart();
}

void transmit()
{
    if (!msg_received) // Если нет сообщения
        return;
    mainTmr.stop();
    byte val = 0x00;
    while (buf.available())
    {
        val = (byte)buf.read();
        Serial.write(val);
        if (val == end_msg)
            break;
    }
    buf.clear();
    directSend = false;
    msg_received = false;
    msg_transmitted = true;
    mainTmr.restart();
}

void calcDelay()
{
    timeoutValue = 0;
    timeoutValue += digitalRead(P1);
    timeoutValue += digitalRead(P2) << 1;
    timeoutValue += digitalRead(P4) << 2;
    timeoutValue += digitalRead(P8) << 3;
    if (timeoutValue == 0)
        timeoutValue = 10;
    timeoutValue *= 20;
    timeoutValue += 1000;
}

void myDelay(unsigned int x)
{
    for (unsigned int i = 0; i <= x; i++)
    {
        delayMicroseconds(1000UL);
    }
}

// Обработка нажатия педали
void buttonClick()
{
    timeoutTmr.restart();
    transmit();
    relayEnable = true;
}

void setup()
{
    directSend = true;
    Serial.begin(9600); // Connect to BSV serial port

    pinMode(P1, INPUT_PULLUP);
    pinMode(P2, INPUT_PULLUP);
    pinMode(P4, INPUT_PULLUP);
    pinMode(P8, INPUT_PULLUP);

    pinMode(PINBTN, INPUT_PULLUP);
    pinMode(RLY, OUTPUT);
    digitalWrite(RLY, HIGH);

    calcDelay();
    timeoutTmr.setTime(timeoutValue);
    button.attachClick(buttonClick);
}

void loop()
{
    while (true)
    {
        button.tick();
        if (msg_received && directSend)
            transmit();
        if (mainTmr.ready())
            receive();
        if (timeoutTmr.ready())
            directSend = true;
    }
    // Плата: Реле
    // Разрешение запуска
    if (relayEnable)
    {
        // Timer ON 400ms
        if (t_on_enable)
        {
            if (flprog::isTimer(t_on_time, 400))
            {
                t_on_out = true;
            }
        }
        else
        {
            t_on_enable = true;
            t_on_time = millis();
        }
    }
    else
    {
        t_on_out = false;
        t_on_enable = false;
    }
    if (t_on_out)
    {
        if (!one_gen_enable)
        {
            one_gen_enable = true;
            one_gen_out = true;
            one_gen_time = millis();
        }
    }
    else
    {
        one_gen_enable = false;
        one_gen_out = false;
    }
    if (one_gen_enable && one_gen_out)
        one_gen_out = !(flprog::isTimer(one_gen_time, 150));
    FTrig_1_Out = 0;
    if ((!(one_gen_out)) && (FTrig_1_OldStat))
    {
        FTrig_1_Out = 1;
    }
    FTrig_1_OldStat = one_gen_out;
    relayEnable = !(FTrig_1_Out);
    digitalWrite(RLY, !one_gen_out);
}
