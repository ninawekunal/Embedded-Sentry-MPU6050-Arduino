#include "Wire.h"
#include "MPU6050.h"

#define NUM_SAMPLES 3
#define NUM_AXES 3

// Since I am taking 3 samples per gesture,
// And I am recording this gesture's 3 set of values.
// Then I will normalize this data using the correlation coff. 
// Corr will be calculated for each between input_data and each of the recorded samples.
// Then I will give vote=1 for each row in corr_input whose value is greater than 0.6
// If the number of votes >=7 , it is a correct gesture.

double recorded_data[10][NUM_SAMPLES * NUM_AXES];
double input_data[NUM_SAMPLES * NUM_AXES];  

double corr_input[10];

double threshold = 14000;

int ledState = LOW;
int readingDone = 1;
boolean goal = false;
int choice;
boolean patternPresent = false;

MPU6050 imu;

boolean showMenu = true;

void setZeroInArray(){
  for(int a=0; a<10; a++){
    for(int b=0; b< NUM_SAMPLES * NUM_AXES; b++){
      recorded_data[a][b] = 0;
      input_data[b]=0;
    }
    corr_input[a] = 0;
  }
}

void setup() {
    Serial.begin(9600);
    imu_setup();
    pinMode(LED_BUILTIN, OUTPUT);
    setZeroInArray();
}

void menu(){
      Serial.println();
      Serial.println("What do you want to do? ");
      Serial.println("1.Record Pattern");
      Serial.println("2.Detect Pattern");
      Serial.println("3.Print recorded values");
      Serial.println("Enter your choice");
      delay(2000);
}

void loop() {
    if(!goal){
      blinkLED();
      if(showMenu){
        menu();
      }
      if (Serial.available() > 0) {
        choice = Serial.parseInt();
        showMenu = false;
        if(choice==1){
          readingDone=0;
          blinkLED();
          start_recording();
          readingDone=1;
          showMenu = true;
        }
        else if(choice==2){
          //record a pattern one time and compare it with the normalized data.
          if(!patternPresent){
            Serial.println();
            Serial.print("Record a pattern first");
          }
          else{
            Serial.println();
            Serial.println("Enter gesture: ");
            detectGesture();
            Serial.println();
            int votes = getVoteCount();
             Serial.print("Votes: ");Serial.print(votes);
             Serial.println();
             if(votes>=7){
                Serial.println();
                Serial.print("DEVICE UNLOCKED");
                Serial.println();
                digitalWrite(LED_BUILTIN, LOW);
             }
             else{
              Serial.println();
              Serial.print("GESTURE INCORRECT");
              Serial.println();
             }
          }
          showMenu = true;
        }
        else if(choice==3){
          //Printing the recorded 30 values each for 10 sample data collected.
          if(patternPresent){
            printRecordedData();
          }
          else{
            Serial.println();
            Serial.print("Pattern not found");
            showMenu = true;
          }
          
        }
        else if(choice==4){
           while(1){}
        }
        else{
          return;
        }
        showMenu = true;
      }
      Serial.println();
      
    }
    delay(2000);
}

// This is used to start recording 10 gestures.
void start_recording(){
      blinkLED();
      double ax, ay, az;
      int i=0;
      Serial.println();
      Serial.print("Enter Reading 1");
      while(1){
        imu_read(&ax, &ay, &az);
        if (!motionDetected(ax, ay, az)) {
             delay(10);
             continue;
        }
        else{
            if(i<10){
              recordIMU(i);
              delay(1000);
              i++;
              Serial.print(" Recorded");
              Serial.println();
              if(i<=9){
                Serial.print("Enter Reading ");Serial.print(i+1);
              }
              else{
                Serial.println("Successfully recorded the pattern");
                patternPresent = true;
                break;
              }
            }
            else{
              // Do nothing
            }
         }
      }
}

// This is used to record samples, which will be used to store 10 sample gestures.
void recordIMU(int j) {
    double ax, ay, az;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
          imu_read(&ax, &ay, &az);
  
          recorded_data[j][i * NUM_AXES + 0] = ax;
          recorded_data[j][i * NUM_AXES + 1] = ay;
          recorded_data[j][i * NUM_AXES + 2] = az;
  
          delay(100);
      }
    
}

// This starts with the detection of gesture recognition algorithm
void detectGesture() {
    double ax, ay, az;
    
    while(1){
      imu_read(&ax, &ay, &az);
      if (!motionDetected(ax, ay, az)) {
         delay(10);
         continue;
      }
      else{
        for (int i = 0; i < NUM_SAMPLES; i++) {
            imu_read(&ax, &ay, &az);
    
            input_data[i * NUM_AXES + 0] = ax;
            input_data[i * NUM_AXES + 1] = ay;
            input_data[i * NUM_AXES + 2] = az;
            Serial.println();
            Serial.print("Please wait..");
            delay(100);
        }
        
        calculateCorrForEachRow();
        break;
      }   
    }
}

// This will show the corr for each row we get from the data.
void calculateCorrForEachRow(){
   for(int x=0; x<10; x++){
      for(int y=0; y<NUM_SAMPLES * NUM_AXES; y++){
          corr_input[x] = correlationCoefficient(input_data, recorded_data[x],y);
      }
   }
}

int getVoteCount(){
  int votes = 0;
  for(int a=0; a<10;a++){
    if(corr_input[a]>0.6){
       votes = votes + 1;
    }
  }
  return votes;
}

// function that returns correlation coefficient. 
double correlationCoefficient(double X[], double Y[], int n) 
{ 
  
    double sum_X = 0, sum_Y = 0, sum_XY = 0; 
    double squareSum_X = 0, squareSum_Y = 0; 
  
    for (int i = 0; i < n; i++) 
    { 
        // sum of elements of array X. 
        sum_X = sum_X + X[i]; 
  
        // sum of elements of array Y. 
        sum_Y = sum_Y + Y[i]; 
  
        // sum of X[i] * Y[i]. 
        sum_XY = sum_XY + X[i] * Y[i]; 
  
        // sum of square of array elements. 
        squareSum_X = squareSum_X + X[i] * X[i]; 
        squareSum_Y = squareSum_Y + Y[i] * Y[i]; 
    } 
  
    // use formula for calculating correlation coefficient. 
    double corr = (double)(n * sum_XY - sum_X * sum_Y)  
                  / sqrt((n * squareSum_X - sum_X * sum_X)  
                      * (n * squareSum_Y - sum_Y * sum_Y)); 
  
    return corr; 
} 

void printRecordedData() {
    const uint16_t numFeatures = NUM_SAMPLES * NUM_AXES ;
    for(int j=0;j<10;j++){
      Serial.println();
      Serial.print("i: ");Serial.println(j);
      Serial.print("aX,aY,aZ");
      Serial.println();
      for (int i = 0; i < numFeatures; i++) {
          Serial.print(recorded_data[j][i]);
          if((i+1)%3==0){
            Serial.println();
          }
          else{
            Serial.print(",");
          }
      }
    }
    Serial.println();
}

bool motionDetected(double ax, double ay, double az) {
    return (ax+ay+az) > 0.7;
}

void imu_setup() {
    Wire.begin();
    imu.initialize();
}

void imu_read(double *ax, double *ay, double *az) {

    *ax = imu.getAccelerationX()/threshold;
    *ay = imu.getAccelerationY()/threshold;
    *az = imu.getAccelerationZ()/threshold - 1.05;
}


void blinkLED(){
    if(readingDone==0){
      if (ledState == LOW) {
        ledState = HIGH;  // Note that this switches the LED *off*
      } else {
        ledState = LOW;  // Note that this switches the LED *on*
      }
      digitalWrite(LED_BUILTIN, ledState);
    }
    else{
      digitalWrite(LED_BUILTIN, HIGH);
    }
}


//      Serial.print("ax");Serial.println(ax);
//      Serial.print("ay");Serial.println(ay);
//      Serial.print("az");Serial.println(az);
//      delay(1000);
