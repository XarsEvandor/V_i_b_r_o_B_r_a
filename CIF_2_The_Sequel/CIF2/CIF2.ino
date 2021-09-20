#include <SPI.h>
#include "SD.h"
#include <Wire.h>
#include "SparkFun_MMA8452Q.h"
#include "arduinoFFT.h"
#include "RTClib.h"
#include <Adafruit_MotorShield.h>

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL 1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL LOG_INTERVAL // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0;             // time of last sync()

#define ECHO_TO_SERIAL 0      // echo data to serial port.
#define WAIT_TO_START 0       // Wait for serial input in setup()
#define MOTOR_TOGGLE 1       // Enable the ability to toggle motor functionality manually.
#define TOGGLE_TIME 0         // Enable the ability to toggle motor functionality in set time intervals.
#define PWM_TIME 0            // Used for Pulse width modulation that is based on the millis factor. This means that the longer the program runs, the slower the motor speed will increase.
#define PWM_PERIODIC 0        // Used for PWM with a periodic change in speed.
#define POWER_PERCENTAGE 0.15 // Used to regulate the speed of the motor easily
#define TIME_INTERVAL 5       // Used to set the number of loops between toggles in TOGGLE_TIME
#define TIMED_ACTIVATION 1    // Used to split the activated time of the program in set timed intervals.
#define ON_TIME 3             // The time that the program will stay active (in hours)
#define ACTIVE_DAYS 2         // The number of days the program will work. Depends on TIMED_ACTIVATION.

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

Sd2Card card;
SdVolume volume;
SdFile root;

RTC_PCF8523 RTC; // define the Real Time Clock object

MMA8452Q accel; // create instance of the MMA8452 class

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

arduinoFFT FFT = arduinoFFT(); // Create FFT object

Adafruit_DCMotor *myMotor = AFMS.getMotor(3); // Select which 'port' M1, M2, M3 or M4. In this case, M3.

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

const uint16_t samples = 256; // always has to be a power of 2, memory does not support 1028+ samples

double samplingFrequency; // should be about 10 times the signal frequency

double xReal[samples];
double xImag[samples];
double yReal[samples];
double yImag[samples];
double zReal[samples];
double zImag[samples];

// the logging file
File logfile;

void error(char *str)
{
    Serial.print("error: ");
    Serial.println(str);

    // red LED indicates error
    digitalWrite(redLEDpin, HIGH);

    while (1)
        ;
}

void setup(void)
{
    pinMode(12, INPUT_PULLUP);
    
    Serial.begin(9600);
    Serial.println();
    Wire.begin();

    // use debugging LEDs
    pinMode(redLEDpin, OUTPUT);
    pinMode(greenLEDpin, OUTPUT);

#if WAIT_TO_START
    Serial.println("Type any character to start");
    while (!Serial.available())
        ;
#endif //WAIT_TO_START

#ifndef ESP8266
    while (!Serial)
        ; // wait for serial port to connect. Needed for native USB
#endif

    // initialize the SD card
    Serial.print("Initializing SD card...");
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(53, OUTPUT);

    // see if the card is present and can be initialized:
    int i = 0;
    while (!SD.begin(chipSelect))
    {
        Serial.print(".");
        delay(100);
        i++;
        if (i > 50)
        {
            Serial.println("Gave up waiting for SD card to mount.");
            return;
        }
    }
    Serial.println("card initialized.");

    // create a new file
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++)
    {
        filename[6] = i / 10 + '0';
        filename[7] = i % 10 + '0';
        if (!SD.exists(filename))
        {
            // only open a new file if it doesn't exist
            logfile = SD.open(filename, FILE_WRITE);
            break; // leave the loop!
        }
    }

    if (!logfile)
    {
        error("couldnt create file");
    }

    Serial.print("Logging to: ");
    Serial.println(filename);

    if (!RTC.begin())
    {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        abort();
    }

    if (!RTC.initialized() || RTC.lostPower())
    {
        Serial.println("RTC is NOT initialized, let's set the time!");
        RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    RTC.start();

    if (!accel.begin())
    { // check accelerometer connection
        error("Accelarometer not connected");
    }
    else
    {
        Serial.println("Accelerometer connected");
    }

//    AFMS.begin();  // create with the default frequency 1.6KHz
    AFMS.begin(1000); // Initiate moror shield at 1000KHz

    // Set the speed to start, from 0 (off) to 255 (max speed). According to Adafruit files, speed here is power given (max is 5v, min is GND)
    myMotor->setSpeed(255 * POWER_PERCENTAGE);
    myMotor->run(FORWARD);

    logfile.println("milliseconds,date-time,X-coordinate,Y-coordinate,Z-coordinate,Orientation,-,walkerX,walkerY,walkerZ,isWalking,-,motorX,motorY,motorZ,motorActive,motorSpeed,activeCounter");
#if ECHO_TO_SERIAL
    Serial.println("milliseconds,date-time,X-coordinate,Y-coordinate,Z-coordinate,Orientation,-,walkerX,walkerY,walkerZ,isWalking,-,motorX,motorY,motorZ,motorActive,motorSpeed,activeCounter");
#endif //ECHO_TO_SERIAL

        logfile.flush();
//        logfile.close();
}

int loopCount = 0; //Variable that will not reset automatically with every loop
int speed = 255 * POWER_PERCENTAGE;
int count = 0;
int sumNew = 0;
int sumOld = 0;
int activeCounter = 0;
int activeCycles = 0;
int sumNewWalking = 0;
int sumOldWalking = 0;
bool toggle = 0;

void loop(void)
{
    //
    //    // create a new file
    //    char filename[] = "LOGGER00.CSV";
    //    for (uint8_t i = 0; i < 100; i++)
    //    {
    //        filename[6] = i / 10 + '0';
    //        filename[7] = i % 10 + '0';
    //        if (!SD.exists(filename))
    //        {
    //            filename[7] = (i-1) % 10 + '0';
    //            Serial.println(filename);
    //            // open the created file and start writting.
    //            logfile = SD.open(filename, FILE_WRITE);
    //            break; // leave the loop!
    //        }
    //    }
    //
    //    if (!logfile)
    //    {
    //        Serial.println("couldnt create file");
    //    }

    //    logfile = SD.open(filename, FILE_WRITE);
    //
    //    if (!logfile)
    //    {
    //        Serial.println("couldnt create file");
    //    }

    DateTime now;

    // delay for the amount of time we want between readings
    delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));
    // delay(LOG_INTERVAL);

    digitalWrite(greenLEDpin, HIGH);

    // log milliseconds since starting
    //    uint32_t m = millis();
    unsigned long m = millis();
    logfile.print(m); // milliseconds since start
    logfile.print(", ");

#if ECHO_TO_SERIAL
    Serial.print(m); // milliseconds since start
    Serial.print(", ");
#endif

    // fetch the time
    now = RTC.now();
    // log time
    //    logfile.print(now.get()); // seconds since 2000
    //    logfile.print(", ");
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(" ");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
#if ECHO_TO_SERIAL
    //    Serial.print(now.get()); // seconds since 2000
    //    Serial.print(", ");
    Serial.print(now.year(), DEC);
    Serial.print("/");
    Serial.print(now.month(), DEC);
    Serial.print("/");
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(":");
    Serial.print(now.minute(), DEC);
    Serial.print(":");
    Serial.print(now.second(), DEC);
#endif //ECHO_TO_SERIAL

logfile.flush();

#if TIMED_ACTIVATION
    // We divide the active time (in millis) by the number of millis in an hour
    // (3.600.000) and we pass it on an integer value so it will be truncated.
    activeCounter = (m / 3600000) - (activeCycles * 24);
    Serial.print("Active counter: ");
    Serial.println(activeCounter);
    myMotor->setSpeed(speed);

    // If the ON_TIME interval has passed, the program will delay for the rest
    // of the day. After ACTIVE_DAYS cycles, the program will exit.
    if (activeCounter >= ON_TIME)
    {
        myMotor->setSpeed(0);
        // delay((24 - ON_TIME) * 1000);
        // myMotor->setSpeed(speed);

        if (activeCounter >= 24)
        {
            activeCycles++;
        }

        // If the program cycles for the required ammount of time, it will stop.
        if (activeCycles == ACTIVE_DAYS)
        {
            myMotor->setSpeed(0);
            logfile.close();
            exit(0);
        }
    }
#endif

    float Xvalue = accel.getCalculatedX();
    float Yvalue = accel.getCalculatedY();
    float Zvalue = accel.getCalculatedZ();

    logfile.print(", "); // change column
    logfile.print(Xvalue);
    logfile.print(", "); // change column
    logfile.print(Yvalue);
    logfile.print(", "); // change column
    logfile.print(Zvalue);

#if ECHO_TO_SERIAL
    Serial.print(", ");
    Serial.print(Xvalue);
    Serial.print(", ");
    Serial.print(Yvalue);
    Serial.print(", ");
    Serial.print(Zvalue);
#endif //ECHO_TO_SERIAL

    logfile.print(", "); // change column
    logfile.print('"');
    if (accel.isRight() == true)
    {
        logfile.print("Pig is on its feet");
    }
    else if (accel.isLeft() == true)
    {
        logfile.print("Pig is on its feet");
    }
    else if (accel.isUp() == true)
    {
        logfile.print("Pig is lying on its left side");
    }
    else if (accel.isDown() == true)
    {
        logfile.print("Pig is lying on its right side");
    }
    else if (accel.isFlat() == true)
    {
        logfile.print("Pig is flat");
    }
    logfile.print('"');

#if ECHO_TO_SERIAL
    Serial.print(", ");
    Serial.print('"');
    if (accel.isRight() == true)
    {
        Serial.print("Pig is on its feet");
    }
    else if (accel.isLeft() == true)
    {
        Serial.print("Pig is on its feet");
    }
    else if (accel.isUp() == true)
    {
        Serial.print("Pig is lying on its left side");
    }
    else if (accel.isDown() == true)
    {
        Serial.print("Pig is lying on its right side");
    }
    else if (accel.isFlat() == true)
    {
        Serial.print("Pig is flat");
    }
    Serial.print('"');
#endif //ECHO_TO_SERIAL

    digitalWrite(greenLEDpin, LOW);

    // // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // // which uses a bunch of power and takes time
    // if ((millis() - syncTime) < SYNC_INTERVAL)
    //     return;
    // syncTime = millis();

    // // blink LED to show we are syncing data to the card & updating FAT!
    // digitalWrite(redLEDpin, HIGH);
    // logfile.flush();
    // digitalWrite(redLEDpin, LOW);

    delay(500);

    logfile.print(", ");
    logfile.print(", ");

#if ECHO_TO_SERIAL
    Serial.print(", ");
    Serial.print(", ");
#endif // ECHO_TO_SERIAL

    get_samples(10.24, 256); // sampling for walking. Using a time_window of 10.24s, each measurement is taken every 40 ms

    samplingFrequency = 100; // setting sampling frequency for fft analysis of walking

    bool walkingBoolean = fftSamples(sumNewWalking, sumOldWalking, 1); // obtain walking frequency

    logfile.print(", "); // change column
    logfile.print(walkingBoolean);

    // blink LED to show we are syncing data to the card & updating FAT!
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(redLEDpin, HIGH);
    logfile.flush();
    digitalWrite(redLEDpin, LOW);

    delay(500); // wait before checking motor

    logfile.print(", "); // change column

    int incomingByte = -1; // for incoming serial data

#if TOGGLE_TIME
    if (count <= 5)
    {
        myMotor->setSpeed(speed * int(toggle));
        //        Serial.print("Speed: ");
        //        Serial.println(speed * int(toggle));
        //        Serial.print("Count: ");
        //        Serial.println(count);
        count++;
    }
    else
    {
        toggle = !toggle;
        count = 0;
    }
#endif

#if MOTOR_TOGGLE //Type 0 to turn off the motor and 1 to turn it on
    myMotor->setSpeed(speed);

    if (Serial.available() > 0)
    {
        // read the incoming byte:
        incomingByte = Serial.read();

        //INPUT IS TRANSLATED IN ASCII. 0 = 48, 1 = 49, END OF TRANSMISSION = 10
        if (incomingByte == 48)
        {
            speed = 0;
            myMotor->setSpeed(speed);
            Serial.println("MOTOR OFF");
        }
        else if (incomingByte == 49)
        {
            speed = 255 * POWER_PERCENTAGE;
            myMotor->setSpeed(speed);
            Serial.println("MOTOR ON");
        }
        else if (incomingByte == 10)
        {
            Serial.println("-------");
        }
        else
            Serial.println("ISSUE");
    }

    Serial.print("Loop: ");
    Serial.print(loopCount);
    Serial.print("   ");
    Serial.print("Speed: ");
    Serial.println(speed);
    Serial.println("");

    loopCount++;
#endif

#if PWM_TIME
    //This snippet is used to figure out how many digits our millis have in order to create PWM in relation to time.
    int count = 0;
    float temp = m;

    while (true)
    {
        if (temp / pow(10, count) < 1)
            break;
        Serial.print("Millis: ");
        Serial.println(m);
        Serial.print("Count: ");
        Serial.print(count);
        Serial.print("   ");
        Serial.print("Temp: ");
        Serial.println(temp / pow(10, count));

        temp = m;
        count++;
    }

    speed = int((m / pow(10, count - 1)) * 10); //For m=1234, count will be 4, thus this line will devide 1234 with 10^3(1000) and return 1, and then multiply by 10, setting the speed to 10.

    Serial.print("Final Count: ");
    Serial.print(count);
    Serial.print("   ");
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print("   ");
    Serial.print("Loop: ");
    Serial.println(loopCount);
    Serial.println("");
    Serial.println("");

    myMotor->setSpeed(speed);

    loopCount++;

#endif

#if PWM_PERIODIC
    //This snippet is used to figure out how many digits our millis have in order to create a consistent PWM in periodic fashion.
    //The count rises by 1  every time the speed zeros or maxes. When the count is an even number the speed will rise by 10 every loop, else it will fall by the same ammount.
    if (count % 2 == 0)
        speed += 10;
    else
        speed -= 10;

    if (speed == 0 || speed == 255 * POWER_PERCENTAGE)
        count++;

    Serial.print("Count: ");
    Serial.print(count);
    Serial.print("   ");
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print("   ");
    Serial.print("Loop: ");
    Serial.println(loopCount);
    Serial.println("");

    myMotor->setSpeed(speed); //For m=1234, count will be 4, thus this line will devide 1234 with 10^3(1000) and return 1, and then multiply by 10, setting the speed to 10.
#endif

    logfile.print(", "); // change column

    get_samples(0.05, 256); // sampling for motor vibration. Using a time_window of 0.05s, each measurement is taken every 0.19 ms.
                            // remember, the vibration period is 1 ms.

    samplingFrequency = 10000; // setting sampling frequency for fft analysis of motor frequency

    bool motorBoolean = fftSamples(sumNew, sumOld, 0); // obtain motor frequency

    logfile.print(", "); // change column
    logfile.print(motorBoolean);

    logfile.print(", "); // change column
#if TOGGLE_TIME
    logfile.print(speed * toggle);
#else
    logfile.print(speed);
#endif

    logfile.print(", "); // change column
    logfile.print(activeCounter);

    // blink LED to show we are syncing data to the card & updating FAT!
    digitalWrite(greenLEDpin, LOW);
    digitalWrite(redLEDpin, HIGH);
    logfile.flush();
    digitalWrite(redLEDpin, LOW);

    logfile.println();

    digitalWrite(greenLEDpin, LOW);

    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // which uses a bunch of power and takes time
    //    if ((millis() - syncTime) < SYNC_INTERVAL)
    //        return;
    //    syncTime = millis();

    // blink LED to show we are syncing data to the card & updating FAT!
    digitalWrite(redLEDpin, HIGH);
    logfile.flush();
    digitalWrite(redLEDpin, LOW);
    //    logfile.close();

    delay(1000);
}

void get_samples(int time_window, uint16_t samples)
{

    for (uint16_t i = 0; i < samples; i++)
    {
        float Xaccel = accel.getCalculatedX();
        float Yaccel = accel.getCalculatedY();
        float Zaccel = accel.getCalculatedZ();
        xReal[i] = Xaccel;
        yReal[i] = Yaccel;
        zReal[i] = Zaccel;
        xImag[i] = 0.0;
        yImag[i] = 0.0;
        zImag[i] = 0.0;
        delayMicroseconds(int((time_window / samples) * 1000000)); // samples = 256, time_window = 10.24 s for walking, 0.05s for motor
    }
}

bool fftSamples(int &sumNew, int &sumOld, int choice)
{

    FFT.Windowing(xReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
    FFT.Windowing(yReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Windowing(zReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

    FFT.Compute(xReal, xImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.Compute(yReal, yImag, samples, FFT_FORWARD);
    FFT.Compute(zReal, zImag, samples, FFT_FORWARD);

    FFT.ComplexToMagnitude(xReal, xImag, samples); /* Compute magnitudes */
    FFT.ComplexToMagnitude(yReal, yImag, samples);
    FFT.ComplexToMagnitude(zReal, zImag, samples);

    double x = FFT.MajorPeak(xReal, samples, samplingFrequency);
    double y = FFT.MajorPeak(yReal, samples, samplingFrequency);
    double z = FFT.MajorPeak(zReal, samples, samplingFrequency);

    sumNew = x + y + z;
    bool motorBoolean = false;

    if (choice == 0)
    {

        if (sumNew < sumOld + 200 && sumNew > sumOld - 200 && sumNew > 1000) // Choice 0 is for motorActive
        {
            motorBoolean = true;
        }

        sumOld = x + y + z;
    }
    else if (choice == 1)
    {
        if (sumNew < sumOld + 4 && sumNew > sumOld - 4 && sumNew < 10) // Choice 1 is for isWalking
        {
            motorBoolean = true;
        }

        sumOld = x + y + z;
    }
    else
    {
        Serial.print("Error");
    }

    logfile.print(x, 6);
    logfile.print(", "); // change column
    logfile.print(y, 6);
    logfile.print(", "); // change column
    logfile.print(z, 6);

    return motorBoolean;
}
