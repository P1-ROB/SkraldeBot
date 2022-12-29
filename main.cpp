#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>
#include <pid.h>

Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4Encoders enc;
Zumo32U4IMU imu;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

// var til afstands sensor 
uint16_t Brightness[]={0 ,1 ,1 ,1,2 ,2 ,2,2 ,2 ,3};

//int Brightness[]={0 ,1 ,1.4142 ,1.7320 ,2 ,2.2360 ,2.4494,2.6457 ,2.8284 ,3};

//  var til linjefølger
#define NUM_SENSORS 5 // antal pladser i vores sensor array
unsigned int lineSensorValues[NUM_SENSORS]; // array til sensorværdier
//const int LSTV = 200; // line sensor threshold value høj værdi(1000) er sort og lav værdi(ned til 0) er den hvide linje 200


//variable til Gyro
int gyroOffset = 0;
uint16_t gyroLastUpdate = 0;
uint32_t turnAngle = 0;
int16_t turnRate = 0;


//til motor kontrol loop
int i = 0;

// DT til PID regulering
long double DT = 0.025;//0.05

//max output til motor 400 pr datablad
int min_crtl_vel = -400;
int max_crtl_vel = 400;

// max hastighed her givet ved pulser pr 0.005 sekunder
int min_crtl_pos = -64; //-150
int max_crtl_pos = 64; //150

int min_crtl_ang = -64; //-40
int max_crtl_ang = 64; //40


// output fra PID_pos
double wished_vel = 0;

//værdier for PID_Vel_l
int left_pos = 0;
int left_old_pos = 0;
double left_vel = 0; 
double left_crtl = 0;

//værdier for PID_Vel_r
int right_pos = 0;
int right_old_pos = 0;
double right_vel = 0; 
double right_crtl = 0;

Pid PID_Vel_l(DT, min_crtl_vel, max_crtl_vel,50);
Pid PID_Vel_r(DT, min_crtl_vel, max_crtl_vel,50);
Pid PID_Pos(DT, min_crtl_pos, max_crtl_pos,3000);
Pid PID_Ang(DT, min_crtl_ang, max_crtl_ang,10);


void initIMU() {
  imu.init(); 
  imu.enableDefault();
  imu.configureForTurnSensing();
  //kalibrere efter person har fjernet hånden fra robotten
  ledYellow(1);
  delay(1000);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  gyroOffset = total / 1024;
  ledYellow(0);
}

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate()
{
  // Read the measurements from the gyro.
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.07 degrees per second per digit.
  //
  // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 14680064/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 14680064 / 17578125;
}


void calibrateLineSensors()
{
  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

void findLine(uint16_t LSTV){
  while (lineSensorValues[4] > LSTV && lineSensorValues[0] > LSTV){ //Løkken kører hvis begge sensorer ikke kans se stregen
  motors.setSpeeds(200, 200);
  lineSensors.readCalibrated(lineSensorValues);
  turnSensorUpdate();
  Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
  
  if (lineSensorValues[4] < LSTV && lineSensorValues[0] > LSTV){ // hvis højre sensor ser linjen først
    
    while(lineSensorValues[0] > LSTV){ //kan se at venstre sensor ikke er på stregen
      lineSensors.readCalibrated(lineSensorValues);
      turnSensorUpdate();
      Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
      motors.setSpeeds(100, 0);
    }
    while(lineSensorValues[4] > LSTV){ //drejer tilbage hvis højre er kommet for langt, og dermed ikke kan se
      lineSensors.readCalibrated(lineSensorValues);
      turnSensorUpdate();
      Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
      motors.setSpeeds(0, -150);
    }
  }
  else if (lineSensorValues[0] < LSTV && lineSensorValues[4] > LSTV){ // hvis venstre sensor ser linjen først
    
    while(lineSensorValues[4] > LSTV){ // Hvis højre sensor er over noget sort
      
      lineSensors.readCalibrated(lineSensorValues);
      turnSensorUpdate();
      Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
      motors.setSpeeds(0, 100);
    }
    while(lineSensorValues[0] > LSTV){ //Hvis venstre er kørt for langt kommer den tilbage på linjen
      
      lineSensors.readCalibrated(lineSensorValues);
      turnSensorUpdate();
      Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
      motors.setSpeeds(-200, 0);
    }
  }
}
  // kører hjulene ud til de ikke står på linjen mere (lige foran linjen)
  while(lineSensorValues[4] < LSTV){
    motors.setSpeeds(0, -150);
    lineSensors.readCalibrated(lineSensorValues);
    turnSensorUpdate();
    Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
  }
  while(lineSensorValues[0] < LSTV){
    motors.setSpeeds(-200, 0);
    lineSensors.readCalibrated(lineSensorValues);
    turnSensorUpdate();
    Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
  }
  motors.setSpeeds(0,0);
  Serial.println(String(lineSensorValues[0])+ "   " + String(lineSensorValues[4]));
}

void hulemand(uint16_t LSTV){ // kører frem og drejer derefter indtil midterste sensor kan se linjen.
                 // Lad være med at stille yderligere spørgsmål. Hulemanden har sine metoder.
  motors.setSpeeds(200, 200);
  delay(250);
  motors.setSpeeds(0,0);
  lineSensors.readCalibrated(lineSensorValues);
  delay(100);
  motors.setSpeeds(200, -200);
  while(lineSensorValues[2] > LSTV){
    lineSensors.readCalibrated(lineSensorValues);
  }
  motors.setSpeeds(0,0);

}

void followLine(uint16_t LSTV){
  lineSensors.readCalibrated(lineSensorValues);
  motors.setSpeeds(125, 125); //sætter standard hastighed
  while(lineSensorValues[0] > LSTV && lineSensorValues[4] > LSTV){//kører indtil de to yderste sensorer kan se linjen

    lineSensors.readCalibrated(lineSensorValues);

    if (lineSensorValues[3] > LSTV){ //drejer til venstre hvis den højre sensor ikke kan se linjen
      motors.setRightSpeed(200);
      motors.setLeftSpeed(125);
      lineSensors.readCalibrated(lineSensorValues);
    }
    else if (lineSensorValues[1] > LSTV){ //drejer til højre hvis den venstre sensor ikke kan se linjen
      motors.setLeftSpeed(200);
      motors.setRightSpeed(125);
      lineSensors.readCalibrated(lineSensorValues);
    }
    else{                                 //kører lige ud hvis begge sesorer kan se linjen
      motors.setSpeeds(125, 125);
      lineSensors.readCalibrated(lineSensorValues);
    }
  }
  motors.setSpeeds(0, 0);  // stopper motorerne nå robotten finder linjen
}


void pos(int p,int l) {
  int k=0;
  enc.getCountsAndResetLeft();
  enc.getCountsAndResetRight();
  wished_vel = 0;
  left_crtl = 0;
  right_crtl = 0;
  left_pos = 0;
  right_pos = 0;
  left_old_pos = 0;
  right_old_pos = 0;
  left_vel = 0;
  right_vel = 0;
  PID_Vel_l.set_error_sum(0);
  PID_Vel_r.set_error_sum(0);
  do {
    
    while (k < l){
    turnSensorUpdate();
    left_pos = enc.getCountsLeft();
    right_pos = enc.getCountsRight();
    delay(25);
    left_vel = (left_pos-left_old_pos);
    right_vel = (right_pos-right_old_pos);
    left_old_pos = left_pos;
    right_old_pos = right_pos;
    
    PID_Pos.update(p, (left_pos+right_pos)/2, &wished_vel, 200);


    if (p > 0){
      PID_Vel_l.update(wished_vel+0.75,left_vel, &left_crtl,30);
    } else{
      PID_Vel_l.update(wished_vel-2.5,left_vel, &left_crtl,30);
    }
    
    PID_Vel_r.update(wished_vel,right_vel, &right_crtl,30);

    motors.setLeftSpeed(left_crtl);//220
    motors.setRightSpeed(right_crtl);
    Serial.println((((int32_t)turnAngle >> 16) * 360) >> 16);
    Serial.println("pos");
    k++;
    }
    k=0;
    Serial.println(left_vel);
    Serial.println(right_vel);
    Serial.println("");
    Serial.println(left_crtl);
    Serial.println(right_crtl);
    Serial.println("");
    Serial.println("");
    Serial.println("");
  
  }
  while((left_pos+right_pos)/2 -p > 20 || (left_pos+right_pos)/2 -p < -20 );
  Serial.print("left pos:");
  Serial.println(enc.getCountsAndResetLeft());
  Serial.print("right pos:");
  Serial.println(enc.getCountsAndResetRight());
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  delay(100);
}

void angle(int a){
  PID_Vel_l.set_error_sum(0);
  PID_Vel_r.set_error_sum(0);
  float angle_sum = 0;
  do {
  turnSensorUpdate();
  PID_Ang.update(a,(((int32_t)turnAngle >> 16) * 360) >> 16,&wished_vel,15);

  //her opdateres control værdien som sendes til motoerene
  PID_Vel_l.update(-wished_vel,left_vel, &left_crtl,5);
  PID_Vel_r.update(wished_vel,right_vel, &right_crtl,5);
  delay(50);
  // her sendes der besked direkte til mortoere
  motors.setLeftSpeed(left_crtl);
  motors.setRightSpeed(right_crtl);
  Serial.print("ang:");
  Serial.println((((int32_t)turnAngle >> 16) * 360) >> 16);
  } 
  while (((((int32_t)turnAngle >> 16) * 360) >> 16) > a + 1 || ((((int32_t)turnAngle >> 16) * 360) >> 16) < a - 1 );
  //(left_crtl < -1 || left_crtl > 1) && (right_crtl < -1 || right_crtl > 1)
  // internel i while (left_crtl < -1 || left_crtl > 1) && (right_crtl < -1 || right_crtl > 1)
  Serial.print("ang:");
  Serial.println((((int32_t)turnAngle >> 16) * 360) >> 16);
  enc.getCountsAndResetLeft();
  enc.getCountsAndResetRight();
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  for(int i = 0; i < 40; i++){
    delay(50);
    turnSensorUpdate();
    angle_sum += (((int32_t)turnAngle >> 16) * 360) >> 16;

  }
  Serial.println(angle_sum);
  if (angle_sum/40 > a + 1|| angle_sum/40 < a - 1 ){
    angle(a);
  }
  
}

void motot_control(int dist, int ang, int update_cycles){
  angle(ang);
  pos(dist, update_cycles);

}

void afstand() {
  while(1){

  
  lineSensors.emittersOn(); //Vi tænder IR sensorene i bunden af zumoen
  //printer hvor mange pulses som IR sensoren får tilbage
  //læser proximity sensoren
  proxSensors.read();
  
 
  //her afgøres om dåsen er tæt på roboten eller langt væk.
  //dette gøres udfra modta pulses, hvis pulsesne giver 5 til 9 vil ZUMOEN 
  //køre frem, da den detecter at dåsen er langt væk.
  //den printer "drive forward" kun fordi motoren er bollet
  //speeden bliver sat sådan at den kan stoppe IR sensoren ved båndet
      if (5<=proxSensors.countsFrontWithLeftLeds() && proxSensors.countsFrontWithLeftLeds()<=9
        && proxSensors.countsFrontWithRightLeds() >= 5 && proxSensors.countsFrontWithRightLeds() <= 9){
          display.clear();
          motors.setSpeeds(200,200);
          
          display.print("drive forward");
          lineSensors.emittersOff();
          delay(500);
          findLine(125);
          delay(100);
          motors.setSpeeds(-200,-200);
          delay(1150);
          motors.setSpeeds(-200,200);
          delay(400);
          motors.setSpeeds(-200,-200);
          delay(500);
          motors.setSpeeds(0,0);
          break;
          

          
        
      }
      //hvis at den ikke detecter en dåse langt væk vil den see om dåsen kunne være tæt på
      //ved at kigge på om vi får nok pulses tilbage på den højre sensor
      else if (proxSensors.countsFrontWithLeftLeds() >= 7
        && proxSensors.countsFrontWithRightLeds() == 10){
        display.clear();
        display.print("big can");
        delay(900);
        lineSensors.emittersOff();
        initIMU();
        turnSensorReset();
        motot_control(1300,-25,50);
        angle(90);
        findLine(100);
        motot_control(-3600,90,50);
        motot_control(1900,155,50);
        angle(105);
        break;

        delay(2000);
        }
        else{
//display cleares, så der kan printes, hvis der modtages pulses af IR sensoren 
          display.clear();
         
        }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  initIMU();
  turnSensorReset();
  PID_Vel_l.set_kp(4.7);//4.8
  PID_Vel_l.set_ki(2.5);//2.25
  PID_Vel_l.set_kd(0.025);//0.025

  PID_Vel_r.set_kp(4.8);//4.95
  PID_Vel_r.set_ki(2.1);//2.2
  PID_Vel_r.set_kd(0.025);//0.025

  PID_Pos.set_kp(0.45); // 0.95
  PID_Pos.set_ki(1); // 0.8
  PID_Pos.set_kd(0.005); //0.00

  PID_Ang.set_kp(2.75); //2
  PID_Ang.set_ki(3.5); // 2.75
  PID_Ang.set_kd(0.005); //0.005

  // set up af prox sensor 
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels(Brightness, sizeof(Brightness)/2);

  // line time 
  lineSensors.initFiveSensors(); // initialize sensors
  // venter på knappen trykkes
  buttonA.waitForButton();

  //kalibrerer sensorer
  calibrateLineSensors();

  // venter på knap igen
  buttonA.waitForButton();

  delay(500);


}

void loop() {
  lineSensors.readCalibrated(lineSensorValues);
  
  findLine(200);

  hulemand(200);

  followLine(200);

  findLine(200);

  motors.setSpeeds(200,200);
  delay(50);
  motors.setSpeeds(0,0);

  findLine(200);

  motors.setSpeeds(200,200);
  delay(50);
  motors.setSpeeds(0,0);

  afstand();


}