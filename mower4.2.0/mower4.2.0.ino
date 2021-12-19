/*  mower 4.0.0!!
 *  this is made for drag flaps
 *  should record: alt, IMU, pnut in, and other data
 *  also not sure if we want to call it mower anymore
 *  anyways should be cool and good
 *  
 *  Updates with 4.1.0:
 *  added temperature and absolute barometric pressure tracking
 *  as well as some minor fixes and QoL here and there
 */
//TODO: data for FLAP_FUN, add IO comm suppurt

#include <BMP388_DEV.h>
#include <SPI.h>
#include <Bounce2.h>
#include <Servo.h>
#include <SdFat.h>
#include <Buzzer.h>
#include <BMI160Gen.h>
#include <Quaternion.h>

#define BUZZ_ACTIVE 0

#define LAUNCH_ALT 10 // meters, obviously
#define TARGET_ALT 243.84 // not used in test flights
#define PROTECTION_TIMER (25 * 1000)

// FLAP_FUN can either be defined as a regression, or as just a constant return
#define FLAP_FUN(f_alt, f_vel) 0.15 //need data
#define SAFE_TO_FLAP() (abs(accl.j) <= 0.5)
bool safetoflap = false;

// landdetect bool went here, not sure what it's used for
const bool landdetect = false;

#define SCK_PIN 27 //pinss
#define ALT_PIN 10
#define IMU_PIN 9
#define BTN1_PIN 24
#define BTN2_PIN 25
#define SRV0_PIN 35
#define SRV1_PIN 36
#define SRV2_PIN 37
#define SRV3_PIN 38
#define BZZ_PIN 3

//buzzer
Buzzer theBuzz(BZZ_PIN);

//servos
#define FLAP_CLS 157 // val for closed flap, needs to be fixed
#define FLAP_OPN 80 // val for open flap, needs to be fixed
#define FLAP_RNG (FLAP_OPN - FLAP_CLS) // eases later calcs
float flap_pos = 0.f;
Servo servos[4];
int servoOffsets[4] = {0, 0, 3, 5};

//button
Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();

//temporal vals
elapsedMillis flighttime = 0;
elapsedMillis time_last = 0; // used for velocity calc

//baro vals
float alt_uncomp; // raw output
float REF_ALT; // first reading is always off, so it gets removed from...
float alt; // the actual altitude
float alt_last; // used for velocity
float vel; // the velocity
float baro_pres; // barometric pressure, for later algorithm use
float temp; // also for later algorithm use
BMP388_DEV altim(ALT_PIN); // baro object

//IMU vals
#define GRANGE 2000 // deg/sec
#define ARANGE 16 // g's
int araw[3]; // raw data
int graw[3];
Quaternion accl = Quaternion(0, 0, 0, 0); // acceleration "raw" input
Quaternion total = Quaternion(1, 0, 0, 0); // current orientation
Quaternion delta = Quaternion(0, 0, 0, 0); // change in orientation
// add grav at some point, not a goal for now TODO
//conversions for IMU
float convertRawAccl(int raw){
  return (ARANGE * (float)raw / 32768.f);
}
float convertRawGyro(int raw){
  return radians(GRANGE * (float)raw / (32768.f * time_last));
}

//SD and other data
SdFatSdio rocketSD;
File lawn;
char filename[] = "lawn000.csv";

//flight status
enum Status { test, armed, flight, landed };
Status flightstate = test;

void setup() {
  Serial.begin(9600);
  delay(200);
  Serial.println("Booting. Systems are:");

  //buzzer
  //theBuzz.morse("buzz");
  theBuzz.beepNum = 2;

  //altimeter setup
  Serial.print("Altimeter:\t"); // chack if altimeter ok, and setup
  SPI.setSCK(SCK_PIN);
  SPI.begin();
  if(altim.begin(NORMAL_MODE, OVERSAMPLING_X4, 
  OVERSAMPLING_X2, IIR_FILTER_2, TIME_STANDBY_160MS) == 1){
    Serial.println("operational");
    altim.enableFIFO();
    altim.setFIFONoOfMeasurements(1);
    Serial.print("reference altitiude is:\t");
    while(!altim.getAltitude(REF_ALT));
    Serial.println(REF_ALT);
  }else{
    Serial.println("unresponsive");
    #ifdef BUZZ_ACTIVE
    theBuzz.beep(100);
    delay(500);
    theBuzz.morse("alt");
    #endif
    theBuzz.beepNum = 3;
    //while(1); // hang if not responding
  }

  Serial.print("IMU:\t"); // check if IMU ok and setup
  BMI160.begin(IMU_PIN, -1); // since this class is for BMI instead of BMX, it has a different chipID than expected, so we just check it isnt 0
  if(BMI160.getDeviceID() != 0x00){
    Serial.println("operational");
    BMI160.autoCalibrateGyroOffset();
    BMI160.setAccelerometerRange(ARANGE);
    BMI160.setGyroRange(GRANGE);
  }else{
    Serial.println("unresponsive");
    #ifdef BUZZ_ACTIVE
    theBuzz.beep(100);
    delay(500);
    theBuzz.morse("IMU");
    #endif
    theBuzz.beepNum = 3;
    //while(1);
  }
  
  //SD setup
  Serial.print("SD:\t");
  if(rocketSD.begin()){
    Serial.println("operational");
    for(int i = 1; i < 1000; i++){
      //make a new file name
      filename[4] = i/100 + '0';
      filename[5] = i/10  + '0';
      filename[6] = i%10  + '0';
      //test the name
      if(!rocketSD.exists(filename)){
        Serial.print("file is:\t");
        Serial.println(filename);
        #ifdef BUZZ_ACTIVE
        //theBuzz.morse((String)i);
        #endif
        break; // no need to continue
      }
    }
  }else{
    Serial.println("unresponsive");
    #ifdef BUZZ_ACTIVE
    theBuzz.beep(100);
    delay(500);
    theBuzz.morse("SD OP");
    #endif
    theBuzz.beepNum = 3;
    //while(1);
  }

  //start the SD file
  if(!lawn.open(filename, O_WRONLY | O_CREAT | O_EXCL)){
    Serial.println("error accesing SD card");
    #ifdef BUZZ_ACTIVE
    theBuzz.morse("SD WR");
    theBuzz.beepNum = 3;
    #endif
  }
  lawn.println("time, altitude, velocity, wRot, iRot, jRot, kRot, xAccl, yAccl, zAccl, temp, baroPres, flapangle");
  lawn.sync();

  //button setup
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  debouncer1.attach(BTN1_PIN);
  debouncer1.interval(5);
  debouncer2.attach(BTN2_PIN);
  debouncer2.interval(5);

  servos[0].attach(SRV0_PIN);
  servos[1].attach(SRV1_PIN);
  servos[2].attach(SRV2_PIN);
  servos[3].attach(SRV3_PIN);
  for(int pos = 0; pos < 4; pos++){
    servos[pos].write(FLAP_CLS + servoOffsets[pos]);
  }

  //setup complete!
  Serial.println("all systems ok. ready for launch! or test!");
  #ifdef BUZZ_ACTIVE
  theBuzz.morse("OK");
  #endif

}

void loop() {
  // flight control loop

  switch(flightstate){
    case test:
      debouncer1.update();
      debouncer2.update();
      if(debouncer1.fell()){
        flap_pos = (flap_pos < 0.8)*(flap_pos + 0.2);
      }
      if(debouncer2.fell()){
        flightstate = armed;
        flighttime = 0;
        flap_pos = 0;
      }

      break;
    
    case armed:
      #ifdef BUZZ_ACTIVE
      theBuzz.updat(); // make sure we're beeping
      #endif
      if(altim.getAltitude(alt_uncomp)){ // get alt if available
        // since the IMU is continous, we will also pick up its data here
        alt = alt_uncomp - REF_ALT;
        
        Serial.println(alt);

        BMI160.readGyro(graw[0], graw[1], graw[3]);
        delta.gyro(convertRawGyro(graw[0]), convertRawGyro(graw[1]), convertRawGyro(graw[2]));
        delta.normalize();
        total.mult(delta);

        BMI160.readAccelerometer(araw[0], araw[1], araw[2]);
        accl = Quaternion(0, convertRawAccl(araw[0]), convertRawAccl(araw[1]), convertRawAccl(araw[2]));

        // wait ten secods then calibrate gyro
        if(flighttime < 10.5*1000 && flighttime > 10*1000){
          #ifdef BUZZ_ACTIVE
          theBuzz.beep(100);
          #endif
          BMI160.autoCalibrateGyroOffset();
          Serial.println("calibrated gyros");
        }

        // wait some more and then find "true" orientation
        if(flighttime < 11*1000 && flighttime > 10.5*1000){
          total.fromAngleVec(atan2(accl.j,sqrt(accl.i*accl.i + accl.k*accl.k)) - HALF_PI,
                      atan2(accl.k,accl.i) - HALF_PI, 0);
          Serial.println("oriented");
        }

        if(alt >= LAUNCH_ALT){
          flightstate = flight;
          flighttime = 0;
          altim.setPresOversampling(OVERSAMPLING_X32);
          altim.setTempOversampling(OVERSAMPLING_X2);
          altim.setIIRFilter(IIR_FILTER_8);
          altim.setTimeStandby(TIME_STANDBY_80MS);
          flap_pos = 0;
          theBuzz.off();
        }
        time_last = 0;
      }
      break;
    case flight:
      if(altim.getMeasurements(temp, baro_pres, alt_uncomp)){
        
        alt = alt_uncomp - REF_ALT;
        vel = 1000 * (alt - alt_last)/(flighttime - time_last);

        BMI160.readGyro(graw[0], graw[1], graw[2]);
        delta.gyro(convertRawGyro(graw[0]), convertRawGyro(graw[1]), convertRawGyro(graw[2]));
        delta.normalize();
        total.mult(delta);
        BMI160.readAccelerometer(araw[0], araw[1], araw[2]);
        accl = Quaternion(0, convertRawAccl(araw[0]), convertRawAccl(araw[1]), convertRawAccl(araw[2]));

        safetoflap = safetoflap || SAFE_TO_FLAP();
        flap_pos = safetoflap * FLAP_FUN(alt, vel);
        
        String out = (String)(flighttime / 1000.f) + ',' + (String)alt + ',' + (String)vel + ',' + 
                      (String)total.w + ',' + (String)total.i + ',' + (String)total.j + ',' + (String)total.k
                      + ',' + (String)accl.i + ',' + (String)accl.j + ',' + (String)accl.k + ',' + 
                      (String)temp + ',' + (String)baro_pres + ',' + (String)flap_pos;
        Serial.println(out);
        lawn.println(out);
        lawn.sync();

        alt_last = alt;
        time_last = 0;

        #ifdef PROTECTION_TIMER
          if(flighttime >= PROTECTION_TIMER){
            flap_pos = 0;
          } // could be unbranched
        #endif

        if((alt<=1 && landdetect) || flighttime >= 120*1000){
          flightstate = landed;
          lawn.close();
          theBuzz.beepNum = 1;
        }
      }
      break;

    case landed:
      #ifdef BUZZ_ACTIVE
      theBuzz.updat();
      #endif
      break;
  }

  // use the processed data
  flap_pos = (float)(flap_pos >= 0)*(flap_pos <= 1)*flap_pos + (float)(flap_pos > 1);
  // safety to make sure flap_pos stays within bounds
  // basically says "if within bounds, use flap_pos, elif >1 ret 1, else ret 0"
  
  // put servos where they need to be
  for(int pos = 0; pos < 4; pos++){
    servos[pos].write(FLAP_CLS + ((float)FLAP_RNG * flap_pos) + servoOffsets[pos]);
  }

}
