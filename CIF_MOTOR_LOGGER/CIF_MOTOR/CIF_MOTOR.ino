#include "SD.h"
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_MotorShield.h>
//#include "SparkFun_MMA8452Q.h"
//#include "arduinoFFT.h"

// A simple data logger for the Arduino analog pins
#define LOG_INTERVAL 1000 // mills between entries
#define ECHO_TO_SERIAL 1  // echo data to serial port
#define WAIT_TO_START 0   // Wait for serial input in setup()
#define POWER_PERCENTAGE 0.3 // Used to regulate the speed of the motor easily

//CAREFUL TO INPUT TIME IN 24H FORMAT. I.E 5PM WILL BE 17!!!
#define BEGINNING_TIME 9 // Used to define the beginning time for motor activation. (int 0-23)
#define ENDING_TIME 17 // Used to define the ending time for motor deactivation. (int 0-23)


#define BEGINNING_MINUTES 10 // Used to define the beginning time for motor activation in the minute implementation. (int 0-59)
#define ENDING_MINUTES 11 // Used to define the ending time for motor deactivation in the minute implementation. (int 0-59)

//BE SURE TO SET THIS TO ZERO FOR WORKING WITH HOURS!
#define WANT_MINUTES 0 // Boolean used to switch between hours(0) and minutes(1) for the beggining and ending times.

//WARNING: PLEASE DO NOT EXCEED 2 ACTIVE DAYS FOR RISK OF DEEP BATTERY DISCHARGE!!!
#define ACTIVATION_DAYS 2 //Used to define the number of days the logger and the motor will be active.

// the digital pins that connect to the LEDs
#define redLEDpin 3
#define greenLEDpin 4

Sd2Card card;
SdVolume volume;
SdFile root;


RTC_PCF8523 rtc; // define the Real Time Clock object

DateTime activationTime;

//MMA8452Q accel; // create instance of the MMA8452 class

//arduinoFFT FFT = arduinoFFT(); // Create FFT object

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();


// Select which 'port' M1, M2, M3 or M4. In this case, M3
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

// the flag file
File flagfile;

char savedDateText[50];
char *c;
int index = 0;

int counter = 0;

uint16_t dateArray[6]; //To save the activation date-time

//const uint16_t samples = 256; // always has to be a power of 2, memory does not support 1028+ samples
//
//double samplingFrequency; // should be about 10 times the signal frequency
//
//double xReal[samples];
//double xImag[samples];
//double yReal[samples];
//double yImag[samples];
//double zReal[samples];
//double zImag[samples];

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
        Serial.begin(9600);
        Serial.println();

        
    #ifndef ESP8266
        while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif


    #if WAIT_TO_START
        Serial.println("Type any character to start");
        while (!Serial.available())
            ;
    #endif //WAIT_TO_START


    #ifndef ESP8266
        while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif

        // initialize the SD card
        Serial.print("Initializing SD card...");
        // make sure that the default chip select pin is set to
        // output, even if you don't use it:
        pinMode(53, OUTPUT);

        // see if the card is present and can be initialized:
        int i = 0;
        while (!SD.begin(chipSelect)) {
        Serial.print(".");
        delay(100);
        i++;
        if (i > 50) {
            Serial.println("Gave up waiting for SD card to mount.");
            return;
        }
        }
        Serial.println("card initialized.");

        // Create flag file
        if(SD.exists("FlagFile.txt")){
            flagfile = SD.open("FlagFile.txt", FILE_READ);
            while(flagfile.available()){
            //   char c = flagfile.read();
            //   savedDateText.concat(c);
                while(flagfile.read(c,1)){
                    if(*c != '\n'){
                        savedDateText[index++] = *c;
                    }
                }

            }
            flagfile.close();
        }
        else{
            flagfile = SD.open("FlagFile.txt", FILE_WRITE);
        }

        Serial.println(savedDateText);

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
        Wire.begin();

//        if (accel.begin() == false)
//        {
//            Serial.println("Accelerometer not Connected. Please check connections and read the hookup guide.");
//            // while (1);
//        }

        if (! rtc.begin()) {
          Serial.println("Couldn't find rtc");
          Serial.flush();
          abort();
        }


        if (!rtc.initialized() || rtc.lostPower())
        {
            Serial.println("rtc is NOT initialized, let's set the time!");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

        }

        rtc.start();
//
//        logfile.println("millis,time,motorX,motorY,motorZ,motorActive,motorSpeed");
//#if ECHO_TO_SERIAL
//        Serial.println("millis,time,motorX,motorY,motorZ,motorActive,motorSpeed");
//#endif


        logfile.println("millis,time,motorSpeed");
#if ECHO_TO_SERIAL
        Serial.println("millis,time,motorSpeed");
#endif

        pinMode(redLEDpin, OUTPUT);
        pinMode(greenLEDpin, OUTPUT);


        AFMS.begin();  // create with the default frequency 1.6KHz
        //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
        
        // Set the speed to start, from 0 (off) to 255 (max speed)
        myMotor->setSpeed(255 * POWER_PERCENTAGE);
        myMotor->run(FORWARD);

        activationTime = rtc.now();
        flagfile.print(activationTime.toString("DD MM YYYY hh mm ss"));
        flagfile.close();

        // /* get the first token */
         char* token = strtok(savedDateText, " ");

         index = 0;
         
        // Find any more?
        while(token != NULL) 
         {
            dateArray[index++] = atoi(token);  
            token = strtok(NULL, " ");
         }

         logfile.flush();
}

int sumNew = 0;
int sumOld = 0;
int motorSpeed = 255 * POWER_PERCENTAGE;

void loop(void)
{
         // delay for the amount of time we want between readings
         delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

         DateTime savedDate(dateArray[2], dateArray[1], dateArray[0], dateArray[3], dateArray[4], dateArray[5]);

         // We set both the hours and the minutes to be BEGINNING_TIME but since we only call the hours or minutes at a time it does not bother us.
         DateTime BEGINNING(activationTime.year(), activationTime.month(), activationTime.day(),BEGINNING_TIME, BEGINNING_MINUTES);
         DateTime ENDING(activationTime.year(), activationTime.month(), activationTime.day(),ENDING_TIME, ENDING_MINUTES);

//         DateTime BEGINNING(0,20, 20, 0);
//         DateTime ENDING(0,21, 21,0);

//         Serial.print("IS VALID:  ");
//         Serial.println(savedDate.isValid());

         digitalWrite(greenLEDpin, HIGH);

         DateTime currentTime = rtc.now();
         DateTime future;
         DateTime futureDay;

         if(!savedDate.isValid()){
          futureDay = activationTime + TimeSpan(ACTIVATION_DAYS,0,0,0);
//          Serial.print("FUTURE_DAY: ");
//          Serial.println(futureDay.toString("DD MM YYYY hh mm ss"));
         }
         else{
          futureDay = savedDate + TimeSpan(ACTIVATION_DAYS,0,0,0);
//          Serial.print("FUTURE_DAY: ");
//          Serial.println(futureDay.toString("DD MM YYYY hh mm ss"));
         }

        if(currentTime.day() < futureDay.day()){
          if(WANT_MINUTES){
            if(currentTime.minute() >= BEGINNING.minute() && currentTime.minute() < ENDING.minute()){
              motorSpeed = 255 * POWER_PERCENTAGE;
              myMotor->setSpeed(motorSpeed);
            }
            else{
                motorSpeed = 0;
                myMotor->setSpeed(0);
            }
          }
          else{
            if(currentTime.hour() >= BEGINNING.hour() && currentTime.hour() < ENDING.hour()){
              motorSpeed = 255 * POWER_PERCENTAGE;
              myMotor->setSpeed(motorSpeed);
            }
            else{
                motorSpeed = 0;
                myMotor->setSpeed(0);
            }
          }
        }
        else{
          myMotor->setSpeed(0);
          SD.remove("FlagFile.txt");
          logfile.flush();
          logfile.close();
          while(1);
        }
        

        // log milliseconds since starting
        uint32_t m = millis();
        logfile.print(m); // milliseconds since start
        logfile.print(", ");
    #if ECHO_TO_SERIAL
        Serial.print(m); // milliseconds since start
        Serial.print(", ");
    #endif
        
        // log time
        logfile.print(currentTime.year(), DEC);
        logfile.print("/");
        logfile.print(currentTime.month(), DEC);
        logfile.print("/");
        logfile.print(currentTime.day(), DEC);
        logfile.print(" ");
        logfile.print(currentTime.hour(), DEC);
        logfile.print(":");
        logfile.print(currentTime.minute(), DEC);
        logfile.print(":");
        logfile.print(currentTime.second(), DEC);
    #if ECHO_TO_SERIAL
        Serial.print(currentTime.year(), DEC);
        Serial.print("/");
        Serial.print(currentTime.month(), DEC);
        Serial.print("/");
        Serial.print(currentTime.day(), DEC);
        Serial.print(" ");
        Serial.print(currentTime.hour(), DEC);
        Serial.print(":");
        Serial.print(currentTime.minute(), DEC);
        Serial.print(":");
        Serial.print(currentTime.second(), DEC);
        Serial.println();
    #endif //ECHO_TO_SERIAL

        logfile.print(", "); // change column

//        get_samples(0.05, 256); // sampling for motor vibration. Using a time_window of 0.05s, each measurement is taken every 0.19 ms.
//                                // remember, the vibration period is 1 ms.
//
//        samplingFrequency = 10000; // setting sampling frequency for fft analysis of motor frequency
//
//        bool motorBoolean = fftSamples(sumNew, sumOld, 0); // obtain motor frequency
//
//        logfile.print(", "); // change column
//        logfile.print(motorBoolean);
//        logfile.print(", "); // change column
        logfile.print(motorSpeed);
        logfile.println();

        logfile.flush();

        digitalWrite(greenLEDpin, LOW);

        delay(10000);
}

//void get_samples(int time_window, uint16_t samples)
//{
//
//    for (uint16_t i = 0; i < samples; i++)
//    {
//        float Xaccel = accel.getCalculatedX();
//        float Yaccel = accel.getCalculatedY();
//        float Zaccel = accel.getCalculatedZ();
//        xReal[i] = Xaccel;
//        yReal[i] = Yaccel;
//        zReal[i] = Zaccel;
//        xImag[i] = 0.0;
//        yImag[i] = 0.0;
//        zImag[i] = 0.0;
//        delayMicroseconds(int((time_window / samples) * 1000000)); // samples = 256, time_window = 10.24 s for walking, 0.05s for motor
//    }
//}
//
//bool fftSamples(int &sumNew, int &sumOld, int choice)
//{
//
//    FFT.Windowing(xReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
//    FFT.Windowing(yReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//    FFT.Windowing(zReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//
//    FFT.Compute(xReal, xImag, samples, FFT_FORWARD); /* Compute FFT */
//    FFT.Compute(yReal, yImag, samples, FFT_FORWARD);
//    FFT.Compute(zReal, zImag, samples, FFT_FORWARD);
//
//    FFT.ComplexToMagnitude(xReal, xImag, samples); /* Compute magnitudes */
//    FFT.ComplexToMagnitude(yReal, yImag, samples);
//    FFT.ComplexToMagnitude(zReal, zImag, samples);
//
//    double x = FFT.MajorPeak(xReal, samples, samplingFrequency);
//    double y = FFT.MajorPeak(yReal, samples, samplingFrequency);
//    double z = FFT.MajorPeak(zReal, samples, samplingFrequency);
//
//    sumNew = x + y + z;
//    bool motorBoolean = false;
//
//    if (choice == 0)
//    {
//
//        if (sumNew < sumOld + 200 && sumNew > sumOld - 200 && sumNew > 1000) // Choice 0 is for motorActive
//        {
//            motorBoolean = true;
//        }
//
//        sumOld = x + y + z;
//    }
//    else if (choice == 1)
//    {
//        if (sumNew < sumOld + 4 && sumNew > sumOld - 4 && sumNew < 10) // Choice 1 is for isWalking
//        {
//            motorBoolean = true;
//        }
//
//        sumOld = x + y + z;
//    }
//    else
//    {
//        Serial.print("Error");
//    }
//
//    logfile.print(x, 6);
//    logfile.print(", "); // change column
//    logfile.print(y, 6);
//    logfile.print(", "); // change column
//    logfile.print(z, 6);
//
//    return motorBoolean;
//}
