#include <Arduino.h>
#include <GyverFIFO.h>
#include <TimerMs.h>
#include <OneButton.h>

////////////////Configuration////////////////
#define DEBUG

#ifdef DEBUG
#define BSV_SERIAL Serial
#define PRN_SERIAL Serial
#else
#define BSV_SERIAL Serial2
#define PRN_SERIAL Serial3
#endif

#ifdef DEBUG
const byte stx = 0x32; // (02) Символ начала посылки
const byte etx = 0x39; // (09) Символ конца посылки
// #define ACK 0x36 // (06) Accept from printer
#else
const byte stx = 0x02; // (02) Символ начала посылки
const byte etx = 0x03; // (03) Символ конца посылки
#endif

/////////////////////
// Declaring Delay`s
#define SCAN_PERIOD 10   // Delay to scan in millisec. (10ms., 100Hz.)
#define PULSE_PERIOD 400 // Delay to Blink pulse indicator in millisec. (400ms., 2.5Hz.)

static bool wdg_state = HIGH;
static bool msg_received;

//////////////////////
// Описание контактов
#define WDG 0x0D    // Индикация работы (WATCHDOG) (pin 13)
#define RLY 0x07    // реле эмуляции кнопки    | Output
#define PINBTN 0x05 // Подключена кнопка       | Input

OneButton button(PINBTN, true);
GyverFIFO<byte, 4096> buf;

TimerMs buttonTickTmr(SCAN_PERIOD);
TimerMs receiveTickTmr(SCAN_PERIOD);
TimerMs pulseTickTmr(PULSE_PERIOD);

void pulseTick();
void receiveTick();
void transmit();
void myDelay(unsigned int x);
void relayClick();
void buttonTick();
void buttonClick();

void pulseTick()
{
    wdg_state = !wdg_state;
    digitalWrite(WDG, wdg_state);
}

void receiveTick()
{
    if (!BSV_SERIAL.available())
        return;
    receiveTickTmr.stop();
    byte val = 0x00;
    while (BSV_SERIAL.available())
    {
        val = (byte)BSV_SERIAL.read();
        switch (val)
        {
        case stx:
            buf.clear();
            msg_received = false;
        default:
            buf.write(val);
            break;
        }
    }
    if (val == etx)
        msg_received = true;
    receiveTickTmr.start();
}

void transmit()
{
    while (buf.peek() != etx)
    {
        PRN_SERIAL.write((byte)buf.read());
    }
    PRN_SERIAL.write((byte)buf.read());
#ifdef DEBUG
    PRN_SERIAL.println("#");
#endif
}

void myDelay(unsigned int x)
{
    for (unsigned int i = 0; i <= x; i++)
    {
        delayMicroseconds(1000UL);
    }
}

void relayClick()
{
    myDelay(100);
    digitalWrite(RLY, false);
    myDelay(150);
    digitalWrite(RLY, true);
}

// Сканирование нажатия педали
void buttonTick()
{
    button.tick();
}
// Обработка нажатия педали
void buttonClick()
{
    buttonTickTmr.stop();
    if (msg_received)
        transmit();
    relayClick();
    buttonTickTmr.start();
}

void setup()
{
#ifdef DEBUG
    Serial.begin(9600); // Port for debugging
#else
    BSV_SERIAL.begin(9600); // Connect to BSV  (Module Read)
    PRN_SERIAL.begin(9600); // Connect to Printer (Module Transmit & ACCEPT)
#endif
    pinMode(WDG, OUTPUT);
    pinMode(RLY, OUTPUT);
    pinMode(PINBTN, INPUT_PULLUP);
    digitalWrite(RLY, HIGH);
    button.attachClick(buttonClick);

    buttonTickTmr.attach(*buttonTick);
    buttonTickTmr.start();

    pulseTickTmr.attach(*pulseTick);
    pulseTickTmr.start();
    receiveTickTmr.attach(*receiveTick);
    receiveTickTmr.start();
}

void loop()
{
    while (true)
    {
        receiveTickTmr.tick();
        buttonTickTmr.tick();
        pulseTickTmr.tick();
    }
}
