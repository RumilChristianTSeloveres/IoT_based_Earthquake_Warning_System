// Include Wire Library for I2C
#include <Wire.h>
#include<SoftwareSerial.h>
// Create nex Rx=2, Tx =3
SoftwareSerial bt(2,3);

// Define LCD pinout
//const int  en = 2, rw = 1, rs = 0, d4 = 4, d5 = 5, d6 = 6, d7 = 7, bl = 3;
 
// Define I2C Address - change if reqiuired
const int i2c_addr = 0x3F;
 
//LiquidCrystal_I2C lcd(i2c_addr, en, rw, rs, d4, d5, d6, d7, bl, POSITIVE);
 
// Level LEDs
int levelLED_Safe = 4;
int levelLED_caution = 5;
int levelLED_Xcaution = 6;
int levelLED_danger = 7;
int levelLED_Xdanger = 8;
 
 
 
//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
 
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;
 
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;
float angle_roll_output_pos, angle_pitch_output_pos;
// Setup timers and temp variables
long loop_timer;
int temp;

//Buzzer
const int Buzzer = 13;

// Display counter
int displaycount = 0;

// Ritcher Value
float angle_map_Richter_Scale;

void setup() {
 
  //Start I2C
  Wire.begin();
  
  // Set display type as 16 char, 2 rows
  //lcd.begin(16,2); 
  
  // Set Level LEDs as outputs
  pinMode(levelLED_Safe, OUTPUT);
  pinMode(levelLED_caution, OUTPUT);
  pinMode(levelLED_Xcaution, OUTPUT);
  pinMode(levelLED_danger, OUTPUT);
  pinMode(levelLED_Xdanger, OUTPUT);
  //Set Buzzer
  pinMode(Buzzer, OUTPUT);
  //Bluetooth
  bt.begin(9600);

  
  //Setup the registers of the MPU-6050                                                       
  setup_mpu_6050_registers(); 
  
  //Read the raw acc and gyro data from the MPU-6050 1000 times                                          
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    read_mpu_6050_data(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }
 
  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
  
  // Start Serial Monitor                                                 
  Serial.begin(115200);
  
  // Init Timer 
  loop_timer = micros();                                               
}
 
void loop(){
 
  // Get data from MPU-6050
  read_mpu_6050_data();
     
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  
  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;  
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians                                
  angle_roll += gyro_y * 0.0000611; 
                                     
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle               
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               
  
  //Accelerometer angle calculations
  
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  angle_roll_acc -= 0.0;                                               
 
  if(set_gyro_angles){ 
  
  //If the IMU has been running 
  //Correct the drift of the gyro pitch angle with the accelerometer pitch angle                      
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004; 
    //Correct the drift of the gyro roll angle with the accelerometer roll angle    
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        
  }
  else{ 
    //IMU has just started  
    //Set the gyro pitch angle equal to the accelerometer pitch angle                                                           
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle                                       
    angle_roll = angle_roll_acc;
    //Set the IMU started flag                                       
    set_gyro_angles = true;                                            
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1; 
  //Take 90% of the output roll value and add 10% of the raw roll value 
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1; 
  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop  
  
  // Print to Serial Monitor   
  //Serial.print(" | Angle  = "); Serial.println(angle_pitch_output);
  angle_pitch_output_pos =  abs(angle_pitch_output);
  // Ritcher Scale
  angle_map_Richter_Scale = (angle_pitch_output_pos*10)/20;
  // Increment the display counter
  displaycount = displaycount +1;
  
  if (displaycount > 100) {
 
  //lcd.clear();
  // Print on first row of LCD
  // lcd.setCursor(0,0);
  // lcd.print("Pitch: ");
  //Serial.println(String(angle_map_Richter_Scale));
  
  
  // lcd.setCursor(0,1);
  // lcd.print("Roll: ");
  // lcd.print(angle_roll_output);
  
  
  // Check Angle for Level LEDs
  
    if (angle_map_Richter_Scale < 2.00) {
    // Turn on Level LED
    digitalWrite(levelLED_Safe, HIGH);
    digitalWrite(levelLED_caution, LOW);
    digitalWrite(levelLED_Xcaution, LOW);
    digitalWrite(levelLED_danger, LOW);
    digitalWrite(levelLED_Xdanger, LOW);
    bt.println(angle_map_Richter_Scale);
    Serial.println(angle_map_Richter_Scale);
    Serial.println("<---Safe--->");
    delay(600);
    } else if ((angle_map_Richter_Scale >= 2.00) && (angle_map_Richter_Scale <=3.00)) {
    // Turn on Level LED
    digitalWrite(levelLED_Safe, LOW);
    digitalWrite(levelLED_caution, HIGH);
    digitalWrite(levelLED_Xcaution, LOW);
    digitalWrite(levelLED_danger, LOW);
    digitalWrite(levelLED_Xdanger, LOW);
    tone(Buzzer,1000,200);
    bt.println(angle_map_Richter_Scale);
    Serial.println(angle_map_Richter_Scale);
    Serial.println("<---Caution--->");
    delay(600);
    } else if ((angle_map_Richter_Scale >= 4.00) && (angle_map_Richter_Scale <=5.00)) {
    // Turn on Level LED
    digitalWrite(levelLED_Safe, LOW);
    digitalWrite(levelLED_caution, LOW);
    digitalWrite(levelLED_Xcaution, HIGH);
    digitalWrite(levelLED_danger, LOW);
    digitalWrite(levelLED_Xdanger, LOW);
    tone(Buzzer,2000,1000);
    bt.println(angle_map_Richter_Scale);
    Serial.println(angle_map_Richter_Scale);
    Serial.println("<---Extreme Caution--->");
    delay(600);
    } else if ((angle_map_Richter_Scale >= 6.00) && (angle_map_Richter_Scale <=7.00)) {
    // Turn on Level LED
    digitalWrite(levelLED_Safe, LOW);
    digitalWrite(levelLED_caution, LOW);
    digitalWrite(levelLED_Xcaution, LOW);
    digitalWrite(levelLED_danger, HIGH);
    digitalWrite(levelLED_Xdanger, LOW);
    tone(Buzzer,5000,2000);
    bt.println(angle_map_Richter_Scale);
    Serial.println(angle_map_Richter_Scale);
    Serial.println("<---Danger--->");
    delay(600);
    } else if (angle_map_Richter_Scale >= 8.00) {
    // Turn on Level LED
    digitalWrite(levelLED_Safe, LOW);
    digitalWrite(levelLED_caution, LOW);
    digitalWrite(levelLED_Xcaution, LOW);
    digitalWrite(levelLED_danger, LOW);
    digitalWrite(levelLED_Xdanger, HIGH);
    tone(Buzzer,5000,2000);
    bt.println(angle_map_Richter_Scale);
    Serial.println(angle_map_Richter_Scale);
    Serial.println("<---Extreme Danger--->");
    delay(600);
    }
    
  displaycount = 0;
   
  }
  
 
 while(micros() - loop_timer < 4000); 
 //Reset the loop timer                                
 loop_timer = micros();
  
}

void setup_mpu_6050_registers(){
 
  //Activate the MPU-6050
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
                                              
}
 
void read_mpu_6050_data(){ 
 
  //Read the raw gyro and accelerometer data
 
  //Start communicating with the MPU-6050                                          
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68,14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}
