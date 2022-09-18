#include <StackArray.h>
#include <Servo.h>

#define echoPinS1 11 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinS1 10 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoPinS2 12 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinS2 13 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoPinS3 6 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinS3 7 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoPinS4 5 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPinS4 4 //attach pin D3 Arduino to pin Trig of HC-SR04
#define buzz1 9
#define buzz2 2
#define buzz3 0
#define buzz4 8
#define servoPin 3
int SERVO_DEG = 15;
int DISTANCE_THRESHHOLD = 2500; //mm
int DANGER_DISTANCE_THRESHHOLD = 50; //mm
double VELOCITY_THRESHHOLD = 0.05; //(mm/ms or m/s)
int NUM_OF_DATA = 1; // number of sample data entries we're taking average out of 

Servo myservo;

class DetectedPerson {
  private:
    double d1;
    double d2;
    unsigned long d1Time;
    unsigned long d2Time;
    int servoSec; // 1 ~ 6
    /*
    Quadrant one as example

              6 |
            5   |
          4     |
        3       |
      2         |
    1           |
    ------------|
    */
    int quadrant; // 1 ~ 4
    /*
        1 | 2
      ----|----
        4 | 3
    */

  public:
    void setD1(double d1) {
      this -> d1 = d1;
    }

    double getD1() {
      return this -> d1; 
    }

    void setD2(double d2) {
      this -> d2 = d2;
    }

    double getD2() {
      return this -> d2; 
    }

    void setD1Time(unsigned long d1Time) {
      this -> d1Time = d1Time;
    }

    void setD2Time(unsigned long d2Time) {
      this -> d2Time = d2Time;
    }

    void setServoSec(int servoSec) {
      this -> servoSec = servoSec;
    }

    int getServoSec() {
      return this -> servoSec;
    }

    int getQuadrant() {
      return this -> quadrant;
    }

    void setQuadrant(int quadrant) {
      return this -> quadrant = quadrant;
    }

    double getVelocity() {
      // origin 0---------(+)
      // if d2 >= d1, person moving away
      // velocity will be +
      // if d2 < d1, person moving closer
      // velocity will be -

      double distanceDelta = d2 - d1;
      distanceDelta = distanceDelta > 60 ? distanceDelta : 0;
      return distanceDelta / (int) (d2Time - d1Time);
    }
};

StackArray<DetectedPerson*> stack1 = StackArray<DetectedPerson*>();
StackArray<DetectedPerson*> stack2 = StackArray<DetectedPerson*>();
StackArray<DetectedPerson*> stack3 = StackArray<DetectedPerson*>();
StackArray<DetectedPerson*> stack4 = StackArray<DetectedPerson*>();

void moveServo(int degree) {
    myservo.write(degree);              // tell servo to go to position in variable 'pos'
    delay(15); // waits 15ms for the servo to reach the position
}

double getDataAverage(int num, int trigPin, int echoPin) {
  double data = 0;
  unsigned long duration = 0;
  double distance = 0;
  for(int i = 0; i < num; i++) {
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // store 20 data entry
  data += distance;
  }

  return data/num;
}

void scan(int trigPin, int echoPin, int quadrant, int servoSec, StackArray <DetectedPerson*> &personStack) {
  double average;

  average = getDataAverage(NUM_OF_DATA, trigPin, echoPin);
  // logic for determining whether something is close or not

  if (average < DISTANCE_THRESHHOLD) {
    // init an DetectdPerson object
    DetectedPerson detected = DetectedPerson();
    // init detected person member variables
    detected.setQuadrant(quadrant); //1_4
    detected.setServoSec(servoSec);//1_6
    detected.setD1(average);
    detected.setD1Time(millis());
    personStack.push(&detected);
  }

  if (average < DANGER_DISTANCE_THRESHHOLD) {
    digitalWrite(buzz1, HIGH);
  } else {
    digitalWrite(buzz1, LOW);
  }
  return 0;
}

void check(int trigPin, int echoPin, int servoSection, StackArray <DetectedPerson*> &personStack) {
  int average = getDataAverage(NUM_OF_DATA, trigPin, echoPin);
 DetectedPerson* person = personStack.pop();
  person -> setD2(average);
  person -> setD2Time(millis());
  double velocity = person -> getVelocity();

 if (average < DANGER_DISTANCE_THRESHHOLD) {
    digitalWrite(buzz1, HIGH);
    delay (500);
  } else {
    digitalWrite(buzz1, LOW);
    delay(500);
  }
  
  if (velocity > VELOCITY_THRESHHOLD) {
    Serial.print("velocity -> ");
    Serial.println(velocity);
    // beep repeatly to warn user
    digitalWrite(buzz2, LOW);
    delay(100);
    digitalWrite(buzz2, HIGH);
    delay(100);
    digitalWrite(buzz2, LOW);

  }
}

void start() {
  Serial.print("working");
  // start scanning
  Serial.println("dynamic working");

  // servo start at 7.5 degrees
  for(int i = 1 ; i < 6 ; i++) {
    // get 20 data and calculate average for 4 sensors
    scan(trigPinS1, echoPinS1, 1, i, stack1);
    scan(trigPinS2, echoPinS2, 2, i, stack2);
    scan(trigPinS3, echoPinS3, 3, i, stack3);
    scan(trigPinS4, echoPinS4, 4, i, stack4);
    moveServo(7.5 + SERVO_DEG*i); 
  }

  for(int i = 5 ; i > 0 ; i--) {
    check(trigPinS1, echoPinS1, i, stack1);
    check(trigPinS2, echoPinS2, i, stack2);
    check(trigPinS3, echoPinS3, i, stack3);
    check(trigPinS4, echoPinS4, i, stack4);
     moveServo(82.5 - SERVO_DEG*(5-(i-1))); 
  }                                           
    Serial.print("working");
}


void setup() {

  pinMode(echoPinS1, INPUT);
  pinMode(trigPinS1, OUTPUT);

  pinMode(echoPinS2, INPUT);
  pinMode(trigPinS2, OUTPUT);

  pinMode(echoPinS3, INPUT);
  pinMode(trigPinS3, OUTPUT);

  pinMode(echoPinS4, INPUT);
  pinMode(trigPinS4, OUTPUT);

  pinMode(buzz1, OUTPUT);
  pinMode(buzz2, OUTPUT);
  pinMode(buzz3, OUTPUT);
  pinMode(buzz4, OUTPUT);

  myservo.attach(servoPin);
}
void loop() {
  start();
}