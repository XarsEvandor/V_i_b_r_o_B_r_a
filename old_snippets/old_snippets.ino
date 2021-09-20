

//Time shift activation.
if((futureDay.day() > now.day() ){
     //If we activate the arduino at 8 ---> While the rtc says the hour of the day is after 8 and before 16(8+8) the motor will work.
     if((futureHour.hour() > now.hour() && activationTime.hour() < now.hour()){
         myMotor->setSpeed(255 * POWER_PERCENTAGE);
     }
     else{
         myMotor->setSpeed(0);
     }
 }
 else{
     myMotor->setSpeed(0);
     SD.remove("FlagFile.txt");
 }


//Time logging. 

Serial.print("ACTIVATION TIME: ");
Serial.print(activationTime.hour(), DEC);
Serial.print(':');
Serial.print(activationTime.minute(), DEC);
Serial.print(':');
Serial.print(activationTime.second(), DEC);
Serial.println();


Serial.print("CURRENT TIME: ");
Serial.print(currentTime.hour(), DEC);
Serial.print(':');
Serial.print(currentTime.minute(), DEC);
Serial.print(':');
Serial.print(currentTime.second(), DEC);
Serial.println();

Serial.print("SAVED TIME: ");
Serial.print(savedDate.hour(), DEC);
Serial.print(':');
Serial.print(savedDate.minute(), DEC);
Serial.print(':');
Serial.print(savedDate.second(), DEC);
Serial.println();

Serial.print("FUTURE TIME: ");
Serial.print(future.hour(), DEC);
Serial.print(':');
Serial.print(future.minute(), DEC);
Serial.print(':');
Serial.print(future.second(), DEC);
Serial.println();
Serial.println();
Serial.println();
