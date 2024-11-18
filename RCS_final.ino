 //(18,19)=(tx,rx) Serial1
 //(16,17) Serial2
 //(14,15) Serial3

 
#include <SPI.h>
#include <SD.h>

int period = 200;           //5Hz : 200ms

int Thrust_Left = 11;       //pin 11
int Thrust_Right = 5;       //pin 5

float Max_Thrust = 0.6;     //1 thruster  
      //2thruster/Max=1.2/re-calculate 'duty_ratio'
float P_gain = 0.008;      //HWP file
float D_gain = 0.1;      //HWP file

int yaw_desired = 200;       //EBIMU
int yaw_rate_desired = 0;

float distance = 0.075;
float Error_yaw;
float Error_yaw_rate;
float Torque;
float Thrust;  
int deadzone = 5;           //Dead Zone Degree

float duty_ratio=0;         //Initial Condition
unsigned long open_time=0;  //Initial Condition

unsigned long t_s;          //to check the millis value

void(*resetFunc)(void)=0;

//sd card
File dataFile;
int chipSelect= 53;
int seqnr =0;
String fileName;
String dataString;
int counter = 0;
int byteCounter = 0;
long last_write = 0;


//proportional valve & solenoid valve
#define fadePin 3
char valve;
char bvalve;

//ARHS 5V
#define SBUF_SIZE 64
char sbuf[SBUF_SIZE];
signed int sbuf_cnt=0;
float roll;
float pitch;
float yaw;
float roll_rate;
float pitch_rate;
float yaw_rate;

//pid system switch
bool inits=false;


int EBimuAsciiParser(float *item, int number_of_item)
{
  int n,i;
  int rbytes;
  char *addr; 
  int result = 0;
  
  rbytes = Serial3.available();
  for(n=0;n<rbytes;n++)
  {
    sbuf[sbuf_cnt] = Serial3.read();
    if(sbuf[sbuf_cnt]==0x0a)
       {
           addr = strtok(sbuf,",");
           for(i=0;i<number_of_item;i++)
           {
              item[i] = atof(addr);
              addr = strtok(NULL,",");
           }
 
           result = 1;
       }
     else if(sbuf[sbuf_cnt]=='*')
       {   sbuf_cnt=-1;
       }
 
     sbuf_cnt++;
     if(sbuf_cnt>=SBUF_SIZE) sbuf_cnt=0;
  }
  
  return result;
}

void setup() {
  Serial.begin(115200);             
  //Serial.flush();                 
  Serial2.begin(9600);            //20Hz
  Serial2.println("CONNECTED TO BLUETOOTH");
  Serial3.begin(115200);
 
  pinMode(5,OUTPUT);
  digitalWrite(5,LOW);
  
  pinMode(9,OUTPUT); //solenoid valve 1
  pinMode(10,OUTPUT); //solenoid valve 2
  pinMode(Thrust_Left,OUTPUT); //solenoid valve 3
  pinMode(Thrust_Right,OUTPUT); //solenoid valve 4
  digitalWrite(9,LOW);
  digitalWrite(10,LOW);
  digitalWrite(Thrust_Left,LOW);
  digitalWrite(Thrust_Right,LOW);

  //sd card
  if (!SD.begin(chipSelect)) {              // Setup SD Card
   Serial.println("ERROR: SD Card failed or not present! Halted.");
   while (1);
  }
  Serial.println("INFO: SD card initialized");
  seqnr = 1;                              // Check on existing files
  fileName = "DL" + String(seqnr) + ".CSV";
  while(SD.exists(fileName)) {
    seqnr++;
    fileName = "DL" + String(seqnr) + ".CSV";
    delay(200); // Make sure file access is done
  }
  dataFile = SD.open(fileName, FILE_WRITE);   // Create new CSV file with appropriate headers
  dataFile.println("Roll,Pitch,Yaw,Roll_rate,Pitch_rate,Yaw_rate");
  Serial.println("INFO: new data file created, ready for logging.");      // Done with SD Card Init
  Serial.println("Initialization completed");   
}


char a;

void loop() {
  
  //ARHS
  float euler[3];
  if(EBimuAsciiParser(euler, 6))
  {
    roll=euler[0];
    pitch =euler[1];
    yaw = euler[2];
    

    if(yaw < 0)
    {
      yaw += 360;
    }
      
    roll_rate=euler[3];
    yaw_rate=euler[4];
    pitch_rate=euler[5];
    
  }

  if (dataFile && millis() - last_write > 150) 
  {
    counter += 1;
    dataString = createDataString();
    //Serial.println(dataString);
    //Serial2.println(dataString);
    //dataFile.println(dataString);
    byteCounter += dataString.length();
    if (byteCounter > 1024) {
      // Serial.println("INFO: flush buffer to SD card");
      dataFile.flush();
      byteCounter = 0;
    }
  }
  
  //Monitor Valve
   if(Serial.available()>0 || Serial2.available()>0)
   {
   valve=Serial.read();
   bvalve = Serial2.read();
         
    if(valve=='A'|| bvalve == 'A')
    {
      Serial.println("Valve2 opened");
      Serial2.println("Valve2 opened");
      digitalWrite(9,HIGH);
    }

    if(valve=='S'|| bvalve == 'S')
    {
      Serial.println("Valve2 closed");
      Serial2.println("Valve2 closed");
      digitalWrite(9,LOW);      
      }
      
   if(valve=='Q' || bvalve=='Q'){
      Serial.println("Valve 1 opened");
      Serial2.println("Valve 1 opened");
      digitalWrite(10,HIGH);
    }

    if(valve=='W' || bvalve=='W'){
      Serial.println("Valve 1 closed");
      Serial2.println("Valve 1 closed");
      digitalWrite(10,LOW);      
      }
      
     if(valve=='Z'|| bvalve=='Z'){
      Serial.println("Valve 3 opened");
      Serial2.println("Valve 3 opened");
      digitalWrite(Thrust_Left,HIGH);
    }

    if(valve=='X' || bvalve=='X'){
      Serial.println("Valve 3 closed");
      Serial2.println("Valve 3 closed");
      digitalWrite(Thrust_Left,LOW);      
      } 
     if(valve=='D' || bvalve=='D'){
      Serial.println("Valve 4 opened");
      Serial2.println("Valve 4 opened");
      digitalWrite(Thrust_Right,HIGH);
     }
     if(valve=='F' || bvalve=='F'){
      Serial.println("Valve 4 closed");
      Serial2.println("Valve 4 closed");
      digitalWrite(Thrust_Right,LOW);
      }

     if(valve== 'g' || bvalve== 'g')
    {
    inits=true;
    Serial.println("CONTROL SYSTEMS STARTS");
    
    }

    if(valve== 'l' || bvalve== 'l')
    {
    inits=false;
    digitalWrite(Thrust_Right,LOW);
    digitalWrite(Thrust_Left,LOW);
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
    Serial.println("ALL VALVES CLOSED");
    
    }

    if(valve=='r' || bvalve =='r')
    {
      resetFunc(); //resets Arduino
      
      }
     
 }
      
     


    //*************************************************//
    //                   PWM Control                   //
    //*************************************************// 
    


 if(inits)
 {  

    Serial.print(roll); Serial.print(","); Serial.print(pitch); Serial.print(",");  Serial.print(yaw); Serial.print(",");
    Serial.print(roll_rate); Serial.print(","); Serial.print(pitch_rate); Serial.print(",");  Serial.println(yaw_rate);
    Serial2.print(roll); Serial2.print(","); Serial2.print(pitch); Serial2.print(","); Serial2.println(yaw); Serial2.print(",");
    Serial2.print(roll_rate); Serial2.print(","); Serial2.print(pitch_rate); Serial2.print(",");  Serial2.println(yaw_rate);
    dataFile.println(dataString);

 
    
    t_s=millis();
    Serial.print("START at ");
    Serial.println(t_s);
    
                                                //OPEN//
      if (duty_ratio > 0)                //Left(YAW<des)
      {
        digitalWrite(Thrust_Left,HIGH); 
        digitalWrite(Thrust_Right,LOW);
        Serial.println("LEFT ");
        Serial2.println("LEFT ");
        
      }
      else if (duty_ratio = 0)                 //for print
      {
        digitalWrite(Thrust_Left,LOW);  //DeadZone(duty=0)
        digitalWrite(Thrust_Right,LOW);
        Serial.println("Dead Zone ");
        Serial2.println("Dead Zone ");
      }
      else
      {
        digitalWrite(Thrust_Left,LOW);   //Right(YAW>des)
        digitalWrite(Thrust_Right,HIGH);
        Serial.println("RIGHT");
        Serial2.println("RIGHT");
      }
      //  Serial.print("OPEN TIME : ");
      //  Serial.println(open_time);

      if (millis() >= t_s+open_time)           //CLOSED//
      {
        digitalWrite(Thrust_Left,LOW);
        digitalWrite(Thrust_Right,LOW);
        
        if (millis() >= t_s+period-5)       //Hz바뀔 때 고려
        {
          Error_yaw = yaw - yaw_desired;
          //Serial.print("Error (yaw) : ");
          //Serial.println(Error_yaw);
          
           
          Error_yaw_rate = yaw_rate - yaw_rate_desired;
          //Serial.print("Error (yaw rate): ");
          //Serial.println(Error_yaw_rate);

              //Duty Ratio & Open Time CALCULATION PART
          if (abs(Error_yaw) < deadzone)   //DEAD ZONE(Yaw)
          {
           digitalWrite(Thrust_Left,LOW);
           digitalWrite(Thrust_Right,LOW);
           Serial.println("Stop! ");
           Serial2.println("Stop! ");
           duty_ratio=0;
           open_time=0;
          }
          else                      //Outside the DEAD ZONE
          { 
            Torque=-P_gain*Error_yaw-D_gain*Error_yaw_rate;
            //Serial.print("Torque : ");
            //Serial.println(Torque);
            Thrust = Torque / distance;
            //Serial.print("Thrust [N] : ");
            //Serial.print(Thrust);

            //*************<duty ratio>***************//
           if (Thrust > Max_Thrust)
           {
            duty_ratio = 100;
           }
           else if (abs(Thrust) < Max_Thrust)
           {
            duty_ratio = 100*Thrust/Max_Thrust;
           }
           else
           {
            duty_ratio = -100;        //계산 시 abs 취할 것
           }
          // Serial.print("duty ratio(%) : ");
           //Serial.println(duty_ratio);

          open_time=abs(duty_ratio)*period/100; //open time
         }
         
         if (millis() < t_s+period)       //t_s+period에 끝
         {
           delay(t_s + period - millis());
         }
        }
        
        else  // t_s+period+5보다 작은 시간일 때 딜레이 후 계산
        {
          delay((t_s+period-5)-millis());
          
          Error_yaw = yaw - yaw_desired;
          //Serial.print("Error (yaw) : ");
          //Serial.println(Error_yaw);
           
          Error_yaw_rate = yaw_rate - yaw_rate_desired;
          //Serial.print("Error (yaw rate): ");
          //Serial.println(Error_yaw_rate);
  
              //Duty Ratio & Open Time CALCULATION PART
          if (abs(Error_yaw) < deadzone)   //DEAD ZONE(YAW)
          {
           digitalWrite(Thrust_Left,LOW);
           digitalWrite(Thrust_Right,LOW);
           Serial.println("Stop! ");
           Serial2.println("Stop! ");
           duty_ratio=0;
           open_time=0;
          }
          else                      //Outside the DEAD ZONE
          { 
            Torque=-P_gain*Error_yaw-D_gain*Error_yaw_rate;
           // Serial.print("Torque : ");
           // Serial.println(Torque);
            Thrust = Torque / distance;
            //Serial.print("Thrust [N] : ");
            //Serial.println(Thrust);

            //*************<duty ratio>***************//
           if (Thrust > Max_Thrust)
           {
            duty_ratio = 100;
           }
           else if (abs(Thrust) < Max_Thrust)
           {
            duty_ratio = 100*Thrust/Max_Thrust;
           }
           else
           {
            duty_ratio = -100; //계산 시 abs 취할 것
           }
          // Serial.print("duty ratio(%) : ");
          // Serial.println(duty_ratio);

          open_time=abs(duty_ratio)*period/100; //open time
         }
         
         if (millis() < t_s+period)        //t_s+period에 끝
         {
           delay(t_s + period - millis());
         }
        }
      }
      else        //현시간이 t_s+open_time보다 짧을경우 딜레이
      {
        delay(t_s+open_time-millis());           //CLOSED//
        
        digitalWrite(Thrust_Left,LOW);
        digitalWrite(Thrust_Right,LOW);

        if (millis() >= t_s+period-5)        //Hz바뀔 때 고려
        {
          Error_yaw = yaw - yaw_desired;
        //  Serial.print("Error (yaw) : ");
        //  Serial.println(Error_yaw);
           
          Error_yaw_rate = yaw_rate - yaw_rate_desired;
        //  Serial.print("Error (yaw rate): ");
        //  Serial.println(Error_yaw_rate);

              //Duty Ratio & Open Time CALCULATION PART
          if (abs(Error_yaw) < deadzone)   //DEAD ZONE(YAW)
          {
           digitalWrite(Thrust_Left,LOW);
           digitalWrite(Thrust_Right,LOW);
           Serial.println("Stop ");
           Serial2.println("Stop ");
           duty_ratio=0;
           open_time=0;
          }
          else                      //Outside the DEAD ZONE
          { 
            Torque=-P_gain*Error_yaw-D_gain*Error_yaw_rate;
          //  Serial.print("Torque : ");
         //   Serial.println(Torque);
            Thrust = Torque / distance;
           // Serial.print("Thrust [N] : ");
           // Serial.println(Thrust);

            //*************<duty ratio>***************//
           if (Thrust > Max_Thrust)
           {
            duty_ratio = 100;
           }
           else if (abs(Thrust)<Max_Thrust)
           {
            duty_ratio = 100*Thrust/Max_Thrust;
           }
           else
           {
            duty_ratio = -100;          //계산 시 abs 취할 것
           }
          // Serial.print("duty ratio(%) : ");
          // Serial.println(duty_ratio);

          open_time=abs(duty_ratio)*period/100; //open time
         }
         
         if (millis() < t_s+period)        //t_s+period에 끝
         {
           delay(t_s + period - millis());
         }
        }
        else  // t_s+period+5보다 작은 시간일 때 딜레이 후 계산
        {
          delay((t_s+period-5)-millis());
          
          Error_yaw = yaw - yaw_desired;
        //  Serial.print("Error (yaw) : ");
        //  Serial.println(Error_yaw);
           
          Error_yaw_rate = yaw_rate - yaw_rate_desired;
        //  Serial.print("Error (yaw rate): ");
        //  Serial.println(Error_yaw_rate);
  
              //Duty Ratio & Open Time CALCULATION PART
          if (abs(Error_yaw) < deadzone)   //DEAD ZONE(YAW)
          {
           digitalWrite(Thrust_Left,LOW);
           digitalWrite(Thrust_Right,LOW);
           Serial.println("Stop! ");
           Serial2.println("Stop! ");
           duty_ratio=0;
           open_time=0;
          }
          else                      //Outside the DEAD ZONE
          { 
            Torque=-P_gain*Error_yaw-D_gain*Error_yaw_rate;
          //  Serial.print("Torque : ");
         //   Serial.println(Torque);
            Thrust = Torque / distance;
            //Serial.print("Thrust [N] : ");
           // Serial.println(Thrust);

            //*************<duty ratio>***************//
           if (Thrust > Max_Thrust)
           {
            duty_ratio = 100;
           }
           else if (abs(Thrust)<Max_Thrust)
           {
            duty_ratio = 100*Thrust/Max_Thrust;
           }
           else
           {
            duty_ratio = -100;         //계산 시 abs 취할 것
           }
          // Serial.print("duty ratio(%) : ");
         // Serial.println(duty_ratio);

          open_time=abs(duty_ratio)*period/100; //open time
         }
         
         if (millis() < t_s+period)       //t_s+period에 끝
         {
           delay(t_s + period - millis());
         }
        }
      }
 }
      
}


void FuncManualAction(){
  String strlnput="";
  String bstrlnput="";
  
  while (Serial.available()>0 ||Serial2.available()>0)
  {
    strlnput += (char) Serial.read();
    bstrlnput += (char) Serial2.read();
    delay(5);
    }

    if(strlnput !="" || bstrlnput!=""){
      int intVal = strlnput.toInt();
      int bintVal = bstrlnput.toInt();
     // analogWriteResolution(5);
      analogWrite(fadePin, intVal);
      Serial.println(strlnput);
      Serial2.println(bstrlnput);

      } 
  }



String createDataString(){

  String ds= "";
  ds += String(roll) + ",";
  ds += String(pitch) + ",";
  ds += String(yaw) + ",";
  ds += String(roll_rate) + ",";
  ds += String(pitch_rate) + ",";
  ds += String (yaw_rate);

  
  return(ds);  
}
