#include <Arduino.h>
#include <GyverButton.h>
#include <GyverTimer.h>
#include <GyverFIFO.h>
// #include <TimerMs.h>
// #include <OneButton.h>

////////////////Configuration////////////////
// #define DEBUG

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
#define SCAN_PERIOD 10 // Delay to scan in millisec. (10ms., 100Hz.)
#define IND_PERIOD 40  // Delay to update indication (40ms., 25Hz.)
#define TIMEOUT 10000  // Timeout for direct transmit message (8000ms., 8s.)

// static byte wdg_state = 0x00;

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
    load,    // B00 000 001 Рестарт Ардуино или сработал таймаут 10 секунд
    empty,   // B00 000 010 Сообщение отсутствует
    message, // B00 000 011 Сообщение принято
    trn,     // B00 000 100 Сообщение отправлено
    btn,     // B00 000 101 Педаль нажата
    rly      // B00 000 110 Реле сработало
};
static enumNodes currentNode = load;
static byte event = 0x0;
static bool newEvent = false;
static bool firstLoop = false;
static bool button_enabled = true;
static bool msg_received = false;

GButton button(PINBTN);
GyverFIFO<byte, 2048> buf;

GTimer indicationTmr(MS);
GTimer receiveTmr(MS);
GTimer timeoutTmr(MS);

void sendIndication();
void msg_receive();
void transmit();
void timeoutHandle();
void myDelay(unsigned int x);
void relayClick();
void buttonClick();

void sendIndication()
{
    newEvent = false;
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
    BSV_SERIAL.print(currentNode);
    BSV_SERIAL.println(event);
    digitalWrite(CLK, LOW);
    digitalWrite(LATCH, LOW);
    shiftOut(DATA, CLK, MSBFIRST, event);
    shiftOut(DATA, CLK, MSBFIRST, mod);
    digitalWrite(LATCH, HIGH);
}

void msg_receive()
{
    if (!BSV_SERIAL.available())
        return;
    receiveTmr.stop();
    timeoutTmr.stop();
    byte val = 0x00;
    while (BSV_SERIAL.available())
    {
        val = (byte)BSV_SERIAL.read();
        if (val == stx)
        {
            buf.clear();
            msg_received = false;
            currentNode = empty;
            newEvent = true;
        }
        buf.write(val);
    }
    if (val == etx)
    {
        msg_received = true;
        currentNode = message;
        newEvent = true;
    }
    if (firstLoop)
        transmit();

    timeoutTmr.start();
    receiveTmr.start();
}

void transmit()
{
    msg_received = false;
    firstLoop = false;
    currentNode = trn;
    newEvent = true;
    while (buf.available())
    {
        PRN_SERIAL.write((byte)buf.read());
        if (buf.peek() == etx)
            break;
    }
#ifdef DEBUG
    PRN_SERIAL.println("#");
#endif
}

void timeoutHandle()
{
    // PRN_SERIAL.println("TimeOut Handler");
    timeoutTmr.stop();
    firstLoop = true;
    if (msg_received)
        transmit();
    currentNode = load;
    newEvent = true;
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

// Обработка нажатия педали
void buttonClick()
{
    button_enabled = false;
    timeoutTmr.stop();
    receiveTmr.stop();

    currentNode = btn;
    newEvent = true;

    if (msg_received)
        transmit();
    relayClick();
    button.resetStates();
    button_enabled = true;
    receiveTmr.start();
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

    pinMode(DATA, OUTPUT);
    pinMode(CLK, OUTPUT);
    pinMode(LATCH, OUTPUT);

    pinMode(WDG, OUTPUT);
    pinMode(PINBTN, INPUT_PULLUP);
    pinMode(RLY, OUTPUT);
    digitalWrite(RLY, HIGH);

    receiveTmr.setInterval(SCAN_PERIOD);
    indicationTmr.setInterval(IND_PERIOD);

    timeoutTmr.setMode(TIMER_TIMEOUT);
    timeoutTmr.setInterval(TIMEOUT);

    newEvent = true;
}

void loop()
{
    while (true)
    {
        if (button_enabled)
            button.tick();

        if (button.isClick())
            buttonClick();

        if (receiveTmr.isReady())
            msg_receive();

        if (timeoutTmr.isReady())
            timeoutHandle();

        if (newEvent && indicationTmr.isReady())
            sendIndication();
    }
}
