#ifndef ARDUINO
  #define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
#define PACKETSIZE 42    // expected DMP packet size (default is 42 bytes)

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class Dynamics{
  public://protected:
  volatile float pos=0, velocity=0, acc=0;

  public: 
  float getPos(){return pos;}
  float getVelocity(){return velocity;}
  float getAcc(){return acc;}
  Dynamics(float p=0, float v=0, float a=0): pos(p), velocity(v), acc(a){};
};

class Sensor: public Dynamics{
  public: 
  float angle();
  void update(){
    auto newPos = angle();
    if (newPos != 1000){
      auto newVelocity = newPos - pos;
      acc = newVelocity - velocity;
      velocity = newVelocity;
      pos = newPos;
    }
  };
};

class MyServo {
  public:
  MyServo();
  void write(int pos);
};

class Engine: public Dynamics{
  int counter = 0;
  Sensor _sensor;
  MyServo _servo;

  public:

  Engine(Dynamics initial, Sensor &sensor, MyServo &servo): Dynamics(initial.pos, initial.velocity, initial.acc){
    _sensor = sensor;
    _servo = servo;
  };

  void update(){
    // Serial.println("tick");
    counter++;
    int d = 144;//constrain(SMALLEST_DELAY + FASTEST_SPEED - abs(velocity), SMALLEST_DELAY, 65535);
    if (counter > d){
      if (acc == 0 &&  velocity == 0 && std::abs(_sensor.getVelocity()) < 0.01 && std::abs(_sensor.getPos()) > 0.01){
        // Serial << "Setting acc" << "\n"; 
        acc = sgn(_sensor.getPos());
      }
      pos = constrain(pos + sgn(velocity), -90, 90);
      // int sum;
      // if (!__builtin_add_overflow(velocity, acc, &sum))
      //   velocity = sum;
      velocity += acc;
      _servo.write(pos + 90);
      // Serial.println(pos);
      // Serial << pos << " " << velocity << " " << acc << " " << counter << "\t\n";
      counter = 0;
    }
  }

  void setAcc(float a){
    acc = a;
  }
};

