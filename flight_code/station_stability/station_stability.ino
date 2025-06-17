#include<Wire.h>
#include<Servo.h>

int ADXL345 = 0x53; // The ADXL345 sensor I2C address

float X_out, Y_out, Z_out;  // Outputs
Servo impeller;
Servo ESC2;
Servo rudder;
unsigned long start = millis();

unsigned long endloop;
//variables for PID control
float target = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;
float min_angle = 45;
float max_angle = 135;
int mtrSpd = 50;
float angle = 0;
struct coord{
  float x;
  float y;
  float z;
  };

int gyro_x, gyro_y, gyro_z;
struct coord assign(float x, float y, float z){
  struct coord coord;
  coord.x = x;
  coord.y = y; 
  coord.z = z;
  return coord;
  } 
struct coord rotation;
//the 'k' values are the ones you need to fine tune before your program will work. Note that these are arbitrary values that you just need to experiment with one at a time.
float Kp = 11;
float Ki = 0.09;
float Kd = 10;


long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;


float acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;
int state = 0;

struct coord gyroMeasure(){
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256;
  Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z_out = Z_out/256;
  rotation = assign(X_out,Y_out,Z_out);
  return rotation;
  }
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  impeller.attach(9, 1000, 2000);
  ESC2.attach(10, 1000, 2000);
  rudder.attach(5, 1000, 2000);
  delay(1000); // Initial pause
  impeller.write(0);
  ESC2.write(0);
  delay(10000);
  Wire.begin(); // Initiate the Wire library
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    struct coord offset;
    offset = gyroMeasure(); 
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += offset.x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += offset.y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += offset.z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                            
  }
 // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;                                                 
  gyro_y_cal /= 1000;                                                 
  gyro_z_cal /= 1000;
}
void loop() {
  // put your main code here, to run repeatedly:
  struct coord gyrovalue;
  gyrovalue = gyroMeasure();
  acc_x = gyrovalue.x;
  acc_y = gyrovalue.y;
  acc_z = gyrovalue.z;
  gyro_x = gyrovalue.x - gyro_x_cal;
  gyro_y = gyrovalue.y - gyro_y_cal;
  gyro_z = gyrovalue.z - gyro_z_cal;
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
  //-----------------done with mpu6050 calibration--------------------------------------// 
  error = target - angle_pitch_output;// proportional
  integral = integral + error; //integral
  derivative = error - last_error; //derivative
  
  
  angle = (error * Kp) + (integral * Ki) + (derivative * Kd); //Might have problem here
  impeller.write(120);
  ESC2.write(mtrSpd);
  rudder.write(90);
  if (angle_pitch_output > 0){
    
    rudder.write(constrain(-abs(angle), min_angle, max_angle));
  }
  else if (angle_pitch_output < 0){
    
    rudder.write(constrain(abs(angle), min_angle, max_angle));
  }
  endloop = millis();
  error = last_error;
  Serial.println(angle_pitch_output);
  if (endloop - start > 25000){
    impeller.write(0);
    ESC2.write(0);
    rudder.write(90); // Center
    while (true); // Halt the program
    }
}
