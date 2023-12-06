#include <Arduino.h>
#include <GyverFIFO.h>
#include <TimerMs.h>
#include <OneButton.h>

////////////////Configuration////////////////
#define DEBUG

#ifdef DEBUG
#define BSV_SERIAL Serial
#define PRN_SERIAL Serial
const byte stx = 0x32; // (02) Символ начала посылки
const byte etx = 0x33; // (03) Символ конца посылки
#else
#define BSV_SERIAL Serial2
#define PRN_SERIAL Serial3
const byte stx = 0x02; // (02) Символ начала посылки
const byte etx = 0x03; // (03) Символ конца посылки
#endif

/////////////////////
// Declaring Delay`s
#define SCAN_PERIOD 10   // Delay to scan in millisec. (10ms., 100Hz.)
#define IND_PERIOD 40    // Delay to update indication (40ms., 25Hz.)
#define PULSE_PERIOD 400 // Delay to Blink pulse indicator in millisec. (400ms., 2.5Hz.)
#define TIMEOUT 8000     // Timeout for direct transmit message (8000ms., 8s.)

static byte wdg_state = 0x00;
static bool msg_received = false;

//////////////////////
// Описание контактов
// Набор контактов для подключения сдвиговых регистров индикации
#define LATCH 0x02 // latch                   | Output
#define CLK 0x03   // clock                   | Output
#define DATA 0x04  // data                    | Output

#define PINBTN 0x05 // Подключена кнопка       | Input
#define WDG 0x0D    // Индикация работы (WATCHDOG) (pin 13)
#define RLY 0x07    // реле эмуляции кнопки    | Output

enum enumNodes
{
    load,    // B00 000 001
    empty,   // B00 000 010
    message, // B00 000 011
    trn,     // B00 000 100
    btn,     // B00 000 101
    rly      // B00 000 110
};
static enumNodes currentNode = empty;
static enumNodes prevNode = load;
static byte event = 0x0;
static bool newEvent = false;
static bool firstLoop = true;

OneButton button(PINBTN, true);
GyverFIFO<byte, 4096> buf;

TimerMs indicationTmr(IND_PERIOD);
TimerMs buttonTickTmr(SCAN_PERIOD);
TimerMs receiveTickTmr(SCAN_PERIOD);
TimerMs pulseTickTmr(PULSE_PERIOD);
TimerMs timeoutTmr(TIMEOUT, true);

void sendIndication();
void indicationTick();
void pulseTick();
void receiveTick();
void transmit();
void timeoutHandle();
void myDelay(unsigned int x);
void relayClick();
void buttonTick();
void buttonClick();

void sendIndication()
{
    if (prevNode == currentNode && !newEvent)
        return;
    indicationTmr.stop();
    newEvent = false;
    prevNode = currentNode;
    event = event << 1;
    if (event == 0x0)
        event++;
    if (event == 0x10)
        event = 0x1;
    byte mod = 0x00;
    switch (currentNode)
    {
    case load:
        mod = 0b001;
        break;
    case empty:
        mod = 0b010;
        break;
    case message:
        mod = 0b011;
        break;
    case trn:
        mod = 0b100;
        break;
    case btn:
        mod = 0b101;
        break;
    case rly:
        mod = 0b110;
        break;
    }
    digitalWrite(CLK, LOW);
    digitalWrite(LATCH, LOW);
    shiftOut(DATA, CLK, MSBFIRST, event);
    shiftOut(DATA, CLK, MSBFIRST, mod);
    digitalWrite(LATCH, HIGH);
    indicationTmr.start();
}

void indicationTick()
{
    sendIndication();
}

void pulseTick()
{
    wdg_state ^= 0x01;
    digitalWrite(WDG, wdg_state);
}

void receiveTick()
{
    receiveTickTmr.stop();
    byte val = 0x00;
    while (BSV_SERIAL.available())
    {
        val = (byte)BSV_SERIAL.read();
        if (val == stx)
        {
            buf.clear();
            msg_received = false;
            currentNode = empty;
        }
        buf.write(val);
    }
    if (val == etx)
    {
        msg_received = true;
        currentNode = message;
        timeoutTmr.start();
    }
    if (firstLoop && msg_received)
        transmit();
    receiveTickTmr.start();
}

void transmit()
{
    currentNode = trn;

    while (buf.available())
    {
        PRN_SERIAL.write((byte)buf.read());
    }
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
    currentNode = rly;
    newEvent = true;
    myDelay(100);
    digitalWrite(RLY, LOW);
    myDelay(150);
    digitalWrite(RLY, HIGH);
}

void timeoutHandle()
{
    firstLoop = true;
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
    currentNode = btn;
    if (msg_received)
    {
        transmit();
    }
    relayClick();
    buttonTickTmr.start();
}

void setup()
{
    firstLoop = true;
#ifdef DEBUG
    Serial.begin(9600); // Port for debugging
#else
    BSV_SERIAL.begin(9600); // Connect to BSV  (Module Read)
    PRN_SERIAL.begin(9600); // Connect to Printer (Module Transmit & ACCEPT)
#endif
    pinMode(WDG, OUTPUT);
    pinMode(RLY, OUTPUT);
    pinMode(PINBTN, INPUT_PULLUP);
    pinMode(DATA, OUTPUT);
    pinMode(CLK, OUTPUT);
    pinMode(LATCH, OUTPUT);
    digitalWrite(RLY, HIGH);
    button.attachClick(buttonClick);

    buttonTickTmr.attach(*buttonTick);
    buttonTickTmr.start();

    timeoutTmr.attach(*timeoutHandle);

    sendIndication();
    indicationTmr.attach(*indicationTick);
    indicationTmr.start();

    pulseTickTmr.attach(*pulseTick);
    pulseTickTmr.start();
    receiveTickTmr.attach(*receiveTick);
    receiveTickTmr.start();
}

void loop()
{
    while (true)
    {
        indicationTmr.tick();
        receiveTickTmr.tick();
        buttonTickTmr.tick();
        pulseTickTmr.tick();
    }
}
