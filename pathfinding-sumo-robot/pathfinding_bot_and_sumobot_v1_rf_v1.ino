// PATHFINDER VARIABLES
volatile int D_mid; // Mid-range distance threshold
volatile int D_max; // Maximum safe distance threshold
volatile int Front_Distance; // Front distance from ultrasonic sensor
volatile int Left_Distance;
volatile int Right_Distance;
volatile int Left_IR_Value; // Value from left IR sensor
volatile int Right_IR_Value; // Value from right IR sensor

void setup() {
    // Common setup code
    Serial.begin(9600);

    // Set the switch pins as input
    pinMode(A0, INPUT_PULLUP);  // Pathfinder
    pinMode(A3, INPUT_PULLUP);  // Sumobot

    // Print initial pin states
    Serial.print("A0 state: ");
    Serial.println(digitalRead(A0));
    Serial.print("A3 state: ");
    Serial.println(digitalRead(A3));

    // Call the appropriate setup function based on the switch state
    if (digitalRead(A0) == LOW) {
        Serial.println("Pathfinder mode selected");
        setupPathfinder();
    } else if (digitalRead(A3) == LOW) {
        Serial.println("Sumobot mode selected");
        setupSumobot();
    } else {
        Serial.println("No mode selected");
    }
}

void loop() {
    // Print pin states periodically for debugging
    Serial.print("A0 state: ");
    Serial.println(digitalRead(A0));
    Serial.print("A3 state: ");
    Serial.println(digitalRead(A3));

    // Debounce delay
    delay(50);

    // Call the appropriate loop function based on the switch state
    if (digitalRead(A0) == LOW) {
        Serial.println("Running Pathfinder loop");
        loopPathfinder();
    } else if (digitalRead(A3) == LOW) {
        Serial.println("Running Sumobot loop");
        loopSumobot();
    } else {
        // Optional: add code for idle state or error handling
        Serial.println("Idle state or no valid mode selected");
        delay(1000); // Adjust delay as necessary
    }
}

void setupPathfinder() {
    delay(500);
    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT); //left ultra trig
    pinMode(8, INPUT); // left ultra echo
    pinMode(10, OUTPUT); // Right ultra trig
    pinMode(11, INPUT); // right ultra echo
    pinMode(12, OUTPUT); // front ultra trig
    pinMode(13, INPUT); // front ultra echo
    D_mid = 20;
    D_max = 400;
    Front_Distance = 0;
    Left_Distance = 0;
    Right_Distance = 0;
    Left_IR_Value = 1;
    Right_IR_Value = 1;
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
}

void loopPathfinder() {
    Obstacle_Avoidance_Main();
}

void Obstacle_Avoidance_Main() {
    Left_IR_Value = digitalRead(A1);
    Right_IR_Value = digitalRead(A2);
    Front_Distance = checkdistanceFront();
    Left_Distance = checkdistanceLeft();
    Right_Distance = checkdistanceRight();

    Serial.print("Front Distance: ");
    Serial.println(Front_Distance);
    Serial.print("Left Distance: ");
    Serial.println(Left_Distance);
    Serial.print("Right Distance: ");
    Serial.println(Right_Distance);

    Infrared_Obstacle_Avoidance();
    Ultrasonic_Obstacle_Avoidance();
}

void Infrared_Obstacle_Avoidance() {
    if (Left_IR_Value == 0 && Right_IR_Value == 1) {
        digitalWrite(2, HIGH);
        analogWrite(5, 125);
        digitalWrite(4, LOW);
        analogWrite(6, 100);
    } else if (Left_IR_Value == 1 && Right_IR_Value == 0) {
        digitalWrite(2, HIGH);
        analogWrite(5, 100);
        digitalWrite(4, LOW);
        analogWrite(6, 125);
    } else {
        digitalWrite(2, HIGH);
        analogWrite(5, 100);
        digitalWrite(4, LOW);
        analogWrite(6, 100);
    }
}

void Ultrasonic_Obstacle_Avoidance() {
  if (Front_Distance <= D_mid) {
  if (Front_Distance <= D_mid && (Left_Distance <= D_mid && Right_Distance <= D_mid )) {
            stopRobot();
            delay(200);
            turnAround();
            delay(310);
            stopRobot();
            delay(200);

            moveForward();
            delay(1000);
            stopRobot();
            delay(200);

            Front_Distance = checkdistanceFront();
            Left_Distance = checkdistanceLeft();
            Right_Distance = checkdistanceRight();
            delay(500);

            if (Front_Distance > D_mid and Front_Distance <= D_max) {
              if (Left_Distance > Right_Distance) {
                turnLeft();
                delay(315);
                stopRobot();
                delay(200);
              } else if (Left_Distance < Right_Distance) {
                turnRight();
                delay(315);
                stopRobot();
                delay(200);
              }

            }


            //}
        }
  if (Left_Distance > Right_Distance ) {
        stopRobot();
        delay(200);
        turnLeft();
        delay(315);
        stopRobot();
        delay(200);

        Front_Distance = checkdistanceFront();
        Left_Distance = checkdistanceLeft();
        Right_Distance = checkdistanceRight();
        delay(500);
    }    
        if (Front_Distance <= D_mid && (Left_Distance < Right_Distance )) {
        stopRobot();
        delay(200);
        turnRight();
        delay(315);
        stopRobot();
        delay(200);

        Front_Distance = checkdistanceFront();
        Left_Distance = checkdistanceLeft();
        Right_Distance = checkdistanceRight();
        delay(500);
    } 
  }
}    

float checkdistanceFront() {
    digitalWrite(12, LOW);
    delayMicroseconds(2);
    digitalWrite(12, HIGH);
    delayMicroseconds(10);
    digitalWrite(12, LOW);
    float distance = pulseIn(13, HIGH) / 58.00;
    delay(10);
    return distance;
}
float checkdistanceLeft() {
    digitalWrite(7, LOW);
    delayMicroseconds(2);
    digitalWrite(7, HIGH);
    delayMicroseconds(10);
    digitalWrite(7, LOW);
    float distance = pulseIn(8, HIGH) / 58.00;
    delay(10);
    return distance;
}
float checkdistanceRight() {
    digitalWrite(10, LOW);
    delayMicroseconds(2);
    digitalWrite(10, HIGH);
    delayMicroseconds(10);
    digitalWrite(10, LOW);
    float distance = pulseIn(11, HIGH) / 58.00;
    delay(10);
    return distance;
}

void stopRobot() {
    digitalWrite(2, LOW);
    analogWrite(5, 0);
    digitalWrite(4, LOW);
    analogWrite(6, 0);
}

void moveForward() {
    digitalWrite(2, HIGH);
    analogWrite(5, 90);
    digitalWrite(4, LOW);
    analogWrite(6, 90);
}

void turnRight() {
    digitalWrite(2, HIGH);
    analogWrite(5, 95);
    digitalWrite(4, HIGH);
    analogWrite(6, 95);
}

void turnLeft() {
    digitalWrite(2, LOW);
    analogWrite(5, 85);
    digitalWrite(4, LOW);
    analogWrite(6, 85);
}

void turnAround() {
    digitalWrite(2, HIGH);
    analogWrite(5, 175);
    digitalWrite(4, HIGH);
    analogWrite(6, 175);
}

// SUMOBOT VARIABLES
volatile int D_mid_Sumo; // Mid-range distance threshold
volatile int D_max_Sumo; // Maximum safe distance threshold
volatile int Front_Distance_Sumo; // Front distance from ultrasonic sensor

unsigned long sumobotStartDelay = 5000; // 5 seconds delay
unsigned long sumobotStartTime; // To store the start time

void moveForwardSumo() {
    digitalWrite(2, HIGH);   // Left wheel forward
    analogWrite(5, 120);     // Left wheel speed
    digitalWrite(4, LOW);    // Right wheel forward
    analogWrite(6, 120);     // Right wheel speed
}

void stopSumo() {
    digitalWrite(2, LOW);
    analogWrite(5, 0);
    digitalWrite(4, LOW);
    analogWrite(6, 0);
    delay(400);
}

void PLUS_ULTRAA() {
    digitalWrite(2, HIGH);
    analogWrite(5, 255);
    digitalWrite(4, LOW);
    analogWrite(6, 255);
}

void turnRightSumo() {
    digitalWrite(2, HIGH);
    analogWrite(5, 110);
    digitalWrite(4, HIGH);
    analogWrite(6, 110);
    delay(350);
}

void Infrared_Tracing() {
    int Left_Tra_Value = 1;
    int Right_Tra_Value = 1;
    int Black = 1;
    Left_Tra_Value = digitalRead(1);
    Right_Tra_Value = digitalRead(9);
    Front_Distance_Sumo = checkdistanceSumo();

    if ((Front_Distance_Sumo > D_mid_Sumo && Front_Distance_Sumo <= D_max_Sumo) && (Left_Tra_Value == Black && Right_Tra_Value == Black)) {
        moveForwardSumo();
    }

    while (true) {
        Front_Distance_Sumo = checkdistanceSumo();
        Left_Tra_Value = digitalRead(7);
        Right_Tra_Value = digitalRead(9);

        if (Front_Distance_Sumo <= D_mid_Sumo && (Left_Tra_Value == Black && Right_Tra_Value == Black)) {
            PLUS_ULTRAA();
        } else if ((Left_Tra_Value != Black && Right_Tra_Value == Black) || (Left_Tra_Value == Black && Right_Tra_Value != Black)) {
            turnRightSumo();
            stopSumo();
            moveForwardSumo();
        } else if (Left_Tra_Value != Black && Right_Tra_Value != Black) {
            turnRightSumo();
            stopSumo();
            moveForwardSumo();
        } else if ((Front_Distance_Sumo > D_mid_Sumo && Front_Distance_Sumo <= D_max_Sumo) && (Left_Tra_Value == Black && Right_Tra_Value == Black)) {
            moveForwardSumo();
        }
    }
}

float checkdistanceSumo() {
    digitalWrite(12, LOW);
    delayMicroseconds(2);
    digitalWrite(12, HIGH);
    delayMicroseconds(10);
    digitalWrite(12, LOW);
    float distance = pulseIn(13, HIGH) / 58.00;
    delay(10);
    return distance;
}

void setupSumobot() {
    Serial.begin(9600);
    pinMode(7, INPUT); // left TCRT5000
    //pinMode(8, INPUT); center TCRT5000
    pinMode(9, INPUT); // right TCRT5000
    pinMode(2, OUTPUT); // left wheel
    pinMode(5, OUTPUT); // left wheel speed
    pinMode(4, OUTPUT); // right wheel
    pinMode(6, OUTPUT); // right wheel speed
    pinMode(12, OUTPUT);
    pinMode(13, INPUT);
    sumobotStartTime = millis(); // Initialize the start time
    D_mid_Sumo = 20;
    D_max_Sumo = 200;
    Front_Distance_Sumo = 0;
}

void loopSumobot() {
    if (millis() - sumobotStartTime >= sumobotStartDelay) {
        Infrared_Tracing();
    } else {
        Serial.println("Waiting for start delay...");
    }
}
