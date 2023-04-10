  // Noah Sarge
  // 4/10/23

  class MotorController{
    class Motor{
      const static int GO_PHASE = 0;
      const static int RAMP_PHASE = 1;
      const static int RAMP_WAIT_PHASE = 2;
      const static int STOP_PHASE = 3;

      const static int STEP_MILLIS = 100; // ms inbetween go calls.

      private:
        int dirPin; // Direction Pin.
        int pwmPin; // Speed Pin.
        int direction; // Direction State.
        double speed; // Speed State.
        
        // Control Variables for the async acceleration.
        int phase; // Controls what work gets done on each update call.
        int last_time; // Time of the last ramp phase.
        int start_time; // Time of the newest acceleration instruction.

        // Acceleration Settings: Variables
        int duration; // How long to accelerate for, from current_speed to new_speed;
        double new_speed; // The new speed to approach.
        double rate; // The rate of change to add to speed each phase.

        /** Private Overload asyncRampSpeed, this gets called each update when phase == RAMP_PHASE
        * @param time: the current time of the motor controller.
        * Sets the time, increments speed by the current rate, which is in relation with the duration and new speed.
        */
        void asyncRampSpeed(long time){        
          // Increment variables.
          this->last_time = time;
          this->speed += rate;
          if(this->last_time - this->start_time >= duration){ // If the duration has been hit, end the ramp speed event.
            this->speed = new_speed;
            this->last_time = 0;
            this->rate = 0;
            this->start_time = 0;
            this->phase = GO_PHASE;
          }
          else{
            this->phase = RAMP_WAIT_PHASE; // If icremented and not done, wait STEP_MILLIS.
          }
        }
      public:
        Motor(int dirPin, int pwmPin){
          this->dirPin = dirPin;
          this->pwmPin = pwmPin;

          this->direction = LOW;
          this->speed = 0;
          
          this->new_speed = 0;
          this->rate = 0;

          this->start_time = 0;
          this->last_time = 0;
          
          this->phase = STOP_PHASE;

          pinMode(dirPin, OUTPUT);
          pinMode(pwmPin, OUTPUT);        
        }
        void setDirection(uint8_t direction){
          this->direction = direction;
        }
        void setSpeed(int speed){
          this->speed = speed;
          this->new_speed = speed;
        }
        /** Public Overloaded asyncRampSpeed
        * @param newSpeed: The new speed to approach.
        * @param duration: The amount of time in ms to approach the new speed.
          Gets called once to set the settings of the motor, each update call will slowly adjuct the current settings to the new ones.
        */
        void asyncRampSpeed(int newSpeed, int duration){
          this->new_speed = newSpeed;
          this->start_time = millis();
          this->duration = duration;
          this->rate = ((this->new_speed - this->speed) / (this->duration / STEP_MILLIS));
          // Serial.print((this->new_speed - this->speed));
          // Serial.print(" / ");
          // Serial.print((this->duration / STEP_MILLIS));
          // Serial.print(" = ");
          // Serial.print(this->rate);
          // Serial.println();
          this->phase = RAMP_PHASE;
        }
        /** update
        * @param time: the current time of the Motor Controller.
        * Calls the corresponding work for the function.
        * Adjusts speed, and calls go unless phase == STOP_PHASE.
        */
        void update(long time){
          if(phase == STOP_PHASE){
            return;
          }
          if(phase == RAMP_PHASE){
            asyncRampSpeed(time);
          }
          else if(phase == RAMP_WAIT_PHASE){
            if(time - last_time > STEP_MILLIS){
              phase = RAMP_PHASE;
            }
          }
          go();
        }
        /** go
          Applies current state as voltage to the motors.
        */
        void go(){
          digitalWrite(dirPin, direction);
          analogWrite(pwmPin, speed);
        }
        void debug(){
          Serial.print(speed);
          Serial.print(", ");
          Serial.print(rate);
          Serial.print(", ");
          Serial.print(last_time);
        }
    };
    private:
      Motor * motorRight;
      Motor * motorLeft;
      int accellDuration;

      void update(long time){
        this->motorLeft->update(time);
        this->motorRight->update(time);
        
        this->motorLeft->debug();
        Serial.print("  |  ");
        this->motorRight->debug();
        Serial.println();
      }
    public:

      MotorController(int dirRPin, int pwmRPin, int dirLPin, int pwmLPin){
        this-> motorRight = new Motor(dirRPin, pwmRPin);
        this-> motorLeft = new Motor(dirLPin, pwmLPin);
        this->accellDuration = 3000;
      }
      void go(){
        update(millis());
        this->motorLeft->go();
        this->motorRight->go();
      }
      void forward(){
      //test Forward
        // setMotor(LOW, LOW, 50,  50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
      
      }
      void backward(){
      //Test Backward
        // setMotor(HIGH, HIGH, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->setSpeed(50);
        this->motorRight->setSpeed(50);
      }
      void spinL(){
      //Test spinL
        // setMotor(HIGH, LOW, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->setSpeed(50);
        this->motorRight->setSpeed(50);
      }
      void spinR(){
      //Test spinR
        // setMotor(LOW, HIGH, 50,50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->setSpeed(50);
        this->motorRight->setSpeed(50);
      }
      void skidL(){
      //Test skidL
        // setMotor(HIGH, LOW, 50, 0);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->setSpeed(50);
        this->motorRight->setSpeed(0);
      }
      void skidR(){
      //Test skidR
        // setMotor(LOW, HIGH, 0, 50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->setSpeed(0);
        this->motorRight->setSpeed(50);
      }
      void brake(){
      //Test brake
        // setMotor(LOW, LOW, 0, 0);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(0, accellDuration);
        this->motorRight->asyncRampSpeed(0, accellDuration);
      }
      void setAccellDuration(int dur){
        this->accellDuration = dur;
      }
  };

  MotorController mc(8, 9, 10, 11);
  String instruction = "";
  int temp;
  void setup() {
    Serial.begin(115200);
  }

  void loop() {
    if(Serial.available() > 0){      
      instruction = Serial.readStringUntil('\n');
      if(instruction == "forward"){
        mc.forward();
      }
      else if(instruction == "stop"){
        mc.brake();
      }
      Serial.println(instruction);
    }
    mc.go();
  }
