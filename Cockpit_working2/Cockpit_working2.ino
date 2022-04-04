// Kids Flight Simulator
//
#include <Servo.h>
#include "tools.h"
#include "LED_Blink.h"
#include "RigidBody2D.h"
#include "vector.h"


#include "SevenSegmentTM1637.h"

//Refuel 4-digit 7 segment Display
const byte Refuel_PIN_CLK = 50;   // Green wire define CLK pin (any digital pin)
const byte Refuel_PIN_DIO = 52;   // Yellow Wire define DIO pin (any digital pin)
SevenSegmentTM1637    refuel_display(Refuel_PIN_CLK, Refuel_PIN_DIO);

//Airspeed 4-digit 7 segment Display
const byte AirSpeed_PIN_CLK = 45;   // Green wire define CLK pin (any digital pin)
const byte AirSpeed_PIN_DIO = 47;   // Yellow Wire define DIO pin (any digital pin)
SevenSegmentTM1637    airspeed_display(AirSpeed_PIN_CLK, AirSpeed_PIN_DIO);

//------------------------------------------------------------------
// Rotary Encoder Inputs
#define CLK 49
#define DT 51
#define SW 53

int counter = 0;
int currentStateCLK;
int lastStateCLK;
int state_DT{0};
int state_SW{0};
String currentDir ="";
unsigned long lastButtonPress = 0;
unsigned long r_prev_time = 0;
//------------------------------------------------------------------


// Gauges servos
Servo servo_RPM1;
Servo servo_RPM2;
Servo servo_Fuel;
Servo servo_Speed;

const int power_swtch{ 2 };
const int gears_swtch{ 15 };
const int f_pump_swtch{ 6 };
const int eng1_start{8};
const int eng2_start{9};
const int eng1_cutoff{10};
const int eng2_cutoff{11};
const uint8_t throttle1_knob{ A0 };
const uint8_t throttle2_knob{ A1 };

int pwr_sw_state{ 0 };
int f_pump_sw_state{ 0 };
int gears_sw_state{ 0 };
int eng1_cut_sw_state{ 0 };
int eng2_cut_sw_state{ 0 };

const int pwr_LED{ 22 };
const int eng_LED{ 23 };
const int fuel_E_LED{ 24 };
const int fuel_L_LED{ 27 };
const int f_pump_LED{ 25 };
const int gears_LED{ 26 };

const int gear_redPin= 3;
const int gear_greenPin = 4;
const int gear_bluePin = 5;

const int fuel_servo{ 30 };
const int rpm1_servo{ 31 };
const int rpm2_servo{ 32 };
const int speed_servo{ 35 };

Fuel_tank tank{10000,0};
int ref_amount{0};
int fill_amount{0};
Engine engine1(tank, eng1_start);
Engine engine2(tank, eng2_start);
LED_timer gear_up;
LED_timer gear_down;
LED_timer gear_over_spd;
LED_timer fuel_low;
LED_timer fuel_empty;

RigidBody airplane{10000.0, engine1, engine2, tank};

double speed_v{ 0 };

unsigned long prev_time = millis();
unsigned long seconds{ 0 };

bool timer_second()
// return true every 1000 millis
{

    if (1000 <= millis() - prev_time) {
        prev_time = millis();
        ++seconds;
        //Serial.println(seconds);

        return true;
    }
    else { return false; }
}

//----------------------------------------------------------------------------
//******************LED BLINKER **********************************************

void LED_timer::end()
{
  prev_on_time=0;
  prev_off_time=0;
  prev_time=0;

  blink_on=false;
  on_interval=false;
  off_interval=false;
  switch_ON=false;
  sequence_done=false;
}

bool LED_timer::blink_LED( int on_int, int off_int, long total)
{
  total_interval=total;
  on_time=on_int;
  off_time=off_int;

  if(sequence_done) return false;
  
  if(!blink_on && !sequence_done) {   // Start here
      blink_on=true;
      prev_on_time=millis();
      prev_time=millis();
      on_interval=true;    
      }
  else {
    if(millis()-prev_time>total_interval) {
    sequence_done=true;
    blink_on=false;
    on_interval=false;
    off_interval=false;
    sequence_done=true;
    return false;
    }
    else{
      if(blink_on) {
        if(on_interval) {
          if(millis()-prev_on_time>on_time) {
            on_interval=false;
            off_interval=true;
            prev_off_time=millis();
            return false;        
          }
          return true;
        }
        else{
          if(millis()-prev_off_time>off_time){
            off_interval=false;
            on_interval=true;
            prev_on_time=millis();
            return true;
          }
          return false;
        }
      }
      else return false;
    }
  }   
}

//----------------------------------------------------------------------------
//****************** FUEL/ENGINE **********************************************

void Fuel_tank::set_refuel(int q) 
{ 
  if(qty+q>cpcty) refuel_qty=cpcty-qty;
  else { 
    refuel_qty=q;
  }
}

void Fuel_tank::refuel()
{
    if(refuel_qty>0) {
      qty+=1;       // lbs
      refuel_qty-=1;
    }    
}

double Engine::get_throttle()
{
    if(get_throttle_axis()== false){
      double angle = floatMap(throttle, 0, 1023, 0, 100);
      return angle;
      //return throttle_value * 100.0 / 1023.0;
    }
    else {
      double angle = floatMap(throttle, 0, 1023, 0, 100);
      return 100-angle;
    }
}

double Engine::rpm()
{
    double flow = fuel_flow();
    if (flow > 0.0 && eng_ON) {               //spools RPM
        if (eng_rpm < 100.0 * flow) {
            eng_rpm += 0.02;             // spools up
        }
        if (eng_rpm > 100.0 * flow) {
            eng_rpm -= 0.02;             // spools down
        }
    }

    if (!eng_ON || flow <= 0 || fuel_cut_off) {
        if(eng_rpm<0) eng_rpm = 0.0;
        else eng_rpm -= 0.04;
        //eng_rpm = 0.0;
    }
    if(eng_rpm>=90) {
      after_burner=true;
    }
    
    else {
      after_burner=false;
    }
    
    return eng_rpm;
}

float Engine::get_thrust()
{
  if(rpm()<=50) thrust=rpm()*0.5;
  else if (50 < rpm() && rpm() <=55 ) thrust=rpm()* 0.6;
  else if (55 < rpm() && rpm() <=60 ) thrust=rpm()* 0.75;
  else if (60 < rpm() && rpm() <=70 ) thrust=rpm()* 0.9;
  else if (70 < rpm() && rpm() <=90 ) thrust=rpm();
  else if (rpm() > 90 ) thrust=rpm()* 1.5;
  else thrust=rpm();
}

bool Engine::fuel_pump()
{
    
    if (f_pump_sw_state && pwr_sw_state) {
        digitalWrite(f_pump_LED, HIGH);
        refuel_display.on();
        return true;
    }
    else {
        digitalWrite(f_pump_LED, LOW);
        eng_ON=false;
        refuel_display.off();
        ref_amount=0;
        return false;
    }
}

double Engine::fuel_flow()
{
    if (fuel_cut_off) {
      f_flow =0.0;
      eng_ON=false;
    }
    
    else if (0 < tank.get_quantity() && eng_ON && fuel_pump() && !fuel_cut_off) {
        f_flow = get_throttle() * gps / 100.0; // gallons
        if (timer_second()) {
          if(after_burner) tank.consume(f_flow*40); // After burner on
          else tank.consume(f_flow*20); // fuel is depleted from tank
        }  
        
        if (tank.get_quantity() <= 0) {
          tank.set_quantity(0.0);
          eng_ON=false;
        }        
    }
    else { f_flow = 0.0; }
    return f_flow;
}

//--------------------------------------------------------------------------------------------

void check_inputs()
{
    pwr_sw_state = digitalRead(power_swtch);
    gears_sw_state = digitalRead(gears_swtch);
    f_pump_sw_state = digitalRead(f_pump_swtch);
}

//---------------------------------
void gauge_pwr()
{
    if (pwr_sw_state == LOW) {
        digitalWrite(pwr_LED, LOW);
        digitalWrite(eng_LED, LOW);
        digitalWrite(fuel_E_LED, LOW);
        digitalWrite(f_pump_LED, LOW);
        refuel_display.off();
        airspeed_display.off();
        
    }
    else {
        digitalWrite(pwr_LED, HIGH);
        airspeed_display.on();
    }
}
//---------------------------------

void gauge_fuel_qty(Fuel_tank& f)
{
    if (pwr_sw_state == HIGH) {
        double lvl = f.get_quantity() * 180 / f.get_capacity();
        double angle = floatMap(lvl, 0, 1023, 0, 895);
        
        servo_Fuel.write(int(angle));
    }
    else servo_Fuel.write(180);
}

//----------
void gauge_refuel(Fuel_tank& f)
{
    if (pwr_sw_state == HIGH) {
        if (f.get_quantity() == 0) {
          if(!fuel_empty.sequence_done){
              if(fuel_empty.blink_LED(700, 700, 7000)) {
                digitalWrite(fuel_E_LED, HIGH);
              }
              else {
                digitalWrite(fuel_E_LED, LOW);
                }
            }
            else {
              digitalWrite(fuel_E_LED, HIGH);
            } 
            digitalWrite(fuel_L_LED, LOW);
            fuel_low.end();
        }

        else if (f.get_quantity() <= 10 * f.get_capacity() / 100) {
            if(!fuel_low.sequence_done){
              if(fuel_low.blink_LED(700, 700, 4200)) {
                digitalWrite(fuel_L_LED, HIGH);
              }
              else {
                digitalWrite(fuel_L_LED, LOW);
                }
            }
            else {
              digitalWrite(fuel_L_LED, HIGH);
            } 
            digitalWrite(fuel_E_LED, LOW);
            fuel_empty.end();
        }

        else if (f.get_quantity() > 0) {
            digitalWrite(fuel_E_LED, LOW);
            digitalWrite(fuel_L_LED, LOW);
            fuel_low.end();
            fuel_empty.end();
        }
        else {}
    } else digitalWrite(fuel_L_LED, LOW);

    // fuel_empty

    //if (rfl_sw_state == HIGH) f.refuel();
    //else {}
}
//--------------------------------------------------------

void gauge_RPM(Engine& e, Servo& servo)
{
    if (pwr_sw_state) {
        double angle = floatMap(e.get_rpm(), 0, 110, 0, 180);
        servo.write(int(angle));
    }
    else servo.write(0.0);
}

//----------
void gauge_eng(Engine& e)
{
    if (e.engineON() == false) {
        digitalWrite(eng_LED, LOW);
    }
    else if (pwr_sw_state && e.engineON() == true) {
        digitalWrite(eng_LED, HIGH);
    }
    else {}
}

//--------------------------------------------------------
void gears()
{
  if (pwr_sw_state == HIGH){
      gears_sw_state=digitalRead(gears_swtch);
    if (gears_sw_state) {      // Gears DOWN
      gear_up.end();
      //gear_over_spd
      if(airplane.vVelocity>250) {
        if(!gear_over_spd.sequence_done){
          if(gear_over_spd.blink_LED(200, 200, 2000000)) {
            setColor(255, 150, 0);
          }
        else {
          setColor(0, 0, 0);
          }
        }
        else {
            setColor(255, 150, 0);
            gear_down.sequence_done=true;
          }    
        //gear_up.switch_ON=false;
        airplane.drag[1].f_status=true;
      }

      else {
        gear_over_spd.end();
        if(!gear_down.sequence_done){
          if(gear_down.blink_LED(700, 700, 7700)) {
            setColor(0, 255, 0);
          }
        else {
          setColor(0, 0, 0);
          }
        }
        else {
            setColor(0, 255, 0);
          }    
        //gear_up.switch_ON=false;
        airplane.drag[1].f_status=true;
      }
      
    }
    else {    // Gears UP
      gear_down.end();
      if(gear_up.blink_LED(700, 700, 7700)) {
        setColor(255, 0, 0);
      }
      else {
        setColor(0, 0, 0);
      }
      airplane.drag[1].f_status=false;   
    }
  }
  else setColor(0, 0, 0);
}

//--------------------------------------------------------
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
double tTime{0};
double tInc{10};          // Time increment touse when stepping through the sim

double s_prev_time;



float RigidBody::total_Thrust()
{
  return engine1.get_thrust()+engine2.get_thrust();
}

void initializeAirplane ()
{
  airplane.vVelocity = 0.0f;
  airplane.vForces = 0.0f;
  //airplane.vMoments=0.0f;
  //airplane.set_fC(0.2);
  //airplane.set_gears_fC(0.05);
  airplane.add_drag_force("fuselage", 0.2, true);   // elem [0]
  airplane.add_drag_force("gears", 0.2, false);   // elem [1]
}

void step_Simulation(float dt)
{
  float F;    // total force
  float A;    // acceleration
  float Vnew; // new velocity at time t + dt
  float Snew; // new position at time t + dt

  

  // calc total Force
  if(airplane.vVelocity<20)  F = (airplane.total_Thrust() - (airplane.total_fC() * airplane.vVelocity) -10) ; // add tire drag
  else F = (airplane.total_Thrust() - (airplane.total_fC() * airplane.vVelocity) ) ;

  // calc acceleration
  A = F/airplane.get_fMass();

  // Calculate the new velocity at time t + dt
  // where V is the velocity at time t
  Vnew = airplane.vVelocity + A * dt;
  
  // Calculate the new displacement at time t + dt
  // where S is the displacement at time t
  Snew = airplane.S + Vnew * dt;
  
  // Update old velocity and displacement with the new ones
  if(Vnew<0) Vnew = 0;
  airplane.vVelocity = Vnew;
  airplane.S = Snew; 

/*
  Serial.print("\t fThrust: ");
  Serial.print(airplane.fThrust);
  Serial.print("\t vVelocity: ");
  Serial.print(airplane.vVelocity);
  Serial.println();
  */
  
}

//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 



void gauge_Speed()
{
    if (pwr_sw_state) {
      if(airplane.vVelocity<=200) {
         double deg = floatMap(airplane.vVelocity, 60, 200, 0, 90);
         servo_Speed.write(deg);
      }
      
      else if(200<airplane.vVelocity && airplane.vVelocity<=300) {
         double deg = floatMap(airplane.vVelocity, 200, 300, 90, 112.5);
         servo_Speed.write(deg);
      }
      else if(300<airplane.vVelocity && airplane.vVelocity<=397) {
         double deg = floatMap(airplane.vVelocity, 300, 397, 112.5, 123.75);
         servo_Speed.write(deg);
      }
      
      else if(300<airplane.vVelocity && airplane.vVelocity<962) {
         double deg = floatMap(airplane.vVelocity, 397, 962, 123.75, 180);
         servo_Speed.write(deg);
      }
      
      /*
        //double deg = speed_v * 180.0 / 500;
        double deg = floatMap(speed_v, 0, 962, 0, 180);
        //Serial.print("\t deg: ");
        //Serial.print(int(deg));
        servo_Speed.write(deg);
        */
        airspeed_display.clear();
        airspeed_display.print(int(airplane.vVelocity));
    }
    else {
      servo_Speed.write(0);
    }
    
}

//--------------------------------------------------------

void Engine::get_readings() {
  
  if(start_btn_state()) eng_ON=true;
  fuel_pump();
  rpm();
}

//*******************************************************************************
//*******************************************************************************
//Refuel Rotary knob

void rotary_setup()
{
    // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);
  initializeAirplane ();
}

void rotary_loop()
{
  int fill{250};
  // Read the current state of CLK
 currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
   if(pwr_sw_state && f_pump_sw_state) {

    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      refuel_display.clear();
      ref_amount-=fill;
      if(ref_amount<0) {
        refuel_display.clear();
        ref_amount=0;
      }
      currentDir ="CCW";
    } else {
      refuel_display.clear();
      // Encoder is rotating CW so increment
      ref_amount+=fill;
      if(ref_amount>9999) {
        ref_amount=9999;
      }
      currentDir ="CW";
    }

    refuel_display.print(ref_amount);
    /*
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Refuel amnt: ");
    Serial.println(ref_amount);
    */
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      //tank.refuel(ref_amount);
      tank.set_refuel(ref_amount);
      ref_amount=0;
      refuel_display.clear();
      refuel_display.print("FILL");
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
    
   }
}

void refuel_DSP_setup()
{
  refuel_display.begin();            // initializes the display
  refuel_display.setBacklight(100);  // set the brightness to 100 %
}

void airspeed_DSP_setup()
{
  airspeed_display.begin();            // initializes the display
  airspeed_display.setBacklight(100);  // set the brightness to 100 %
}


//------------------------------------------------------------------

void setColor(int redValue, int greenValue, int blueValue) 
{
  analogWrite(gear_redPin, redValue);
  analogWrite(gear_greenPin, greenValue);
  analogWrite(gear_bluePin, blueValue);
}

//------------------------------------------------------------------

void print_stats()
{
    /*
    ///Serial.print("\t Time(millis): ");
    //Serial.print(millis());
    //Serial.print("\t Time(s): ");
    //Serial.print(seconds);
    Serial.print("\t Fuel Amount: ");
    Serial.print(tank.get_quantity());
    Serial.print("\t Fuel flow: ");
    Serial.print(engine1.fuel_flow());
Serial.print("\t Fuel Low: ");
    Serial.print(digitalRead(fuel_L_LED));
    
    Serial.print("\t Throttle 1: ");
    Serial.print(engine1.get_throttle());
    Serial.print("\t Throttle 2: ");
    Serial.print(engine2.get_throttle());
    Serial.print("\t RPM 1: ");
    Serial.print(engine1.rpm());
    Serial.print("\t RPM 2: ");
    Serial.print(engine2.rpm());
    Serial.print("\t E1 On: ");
    Serial.print(engine1.engineON());
    Serial.print("\t E2 On: ");
    Serial.print(engine2.engineON());
    //Serial.print("\t Speed: ");
    //Serial.print(speed_v);
    //Serial.print("\t F_pump: ");
    //Serial.print(f_pump_sw_state);
    Serial.println();
    */
    /*
    Serial.print("\t Speed: ");
    Serial.print(speed_v);
    
   
    Serial.print("\t drag: ");
    Serial.print(airplane.total_fC());
    Serial.print("\t name: ");
    Serial.print(airplane.drag[0].f_name);
    Serial.print("\t cD: ");
    Serial.print(airplane.drag[0].cD);
    Serial.print("\t Status: ");
    Serial.print(airplane.drag[0].f_status);

        Serial.print("\t name: ");
    Serial.print(airplane.drag[1].f_name);
    Serial.print("\t cD: ");
    Serial.print(airplane.drag[1].cD);
    Serial.print("\t Status: ");
    Serial.print(airplane.drag[1].f_status);
    Serial.println();
    */

    
    //Serial.println();
    
    
}

void setup()
{
    Serial.begin(9600);
    pinMode(power_swtch, INPUT);
    pinMode(eng1_start, INPUT);
    pinMode(eng2_start, INPUT);
    pinMode(gears_swtch, INPUT);
    pinMode(f_pump_swtch, INPUT);
    pinMode(eng1_cutoff, INPUT);
    pinMode(eng2_cutoff, INPUT);
    pinMode(throttle1_knob, INPUT);
    pinMode(throttle2_knob, INPUT);

    //4-Digit 7 Segment Display
    //pinMode(PIN_CLK, OUTPUT);
    //pinMode(PIN_DIO, OUTPUT);
       
    pinMode(pwr_LED, OUTPUT);
    pinMode(eng_LED, OUTPUT);
    pinMode(fuel_E_LED, OUTPUT);
    pinMode(fuel_L_LED, OUTPUT);
    pinMode(gears_LED, OUTPUT);
    pinMode(f_pump_LED, OUTPUT);
    pinMode(speed_servo, OUTPUT);
    pinMode(rpm1_servo, OUTPUT);
    pinMode(rpm2_servo, OUTPUT);
    pinMode(fuel_servo, OUTPUT);

    pinMode(gear_redPin, OUTPUT);
    pinMode(gear_greenPin, OUTPUT);
    pinMode(gear_bluePin, OUTPUT); 
    
    servo_RPM1.attach(rpm1_servo);
    servo_RPM2.attach(rpm2_servo);
    servo_Fuel.attach(fuel_servo);
    servo_Speed.attach(speed_servo);
    engine2.flip_throttle(true);

    rotary_setup();
    refuel_DSP_setup();
    airspeed_DSP_setup();
    s_prev_time=millis();
}


void loop()
{
    check_inputs();
    timer_second();
    
    engine1.set_throttle(analogRead(throttle1_knob));
    engine2.set_throttle(analogRead(throttle2_knob));
    
    engine1.set_F_cut_off(digitalRead(eng1_cutoff));
    engine2.set_F_cut_off(digitalRead(eng2_cutoff));

    engine1.get_readings();
    engine2.get_readings();

    gears();
    gauge_pwr();
    
    gauge_eng(airplane.engine1);
    gauge_eng(airplane.engine2);
 
    
    gauge_RPM(engine1, servo_RPM1);
    gauge_RPM(engine2, servo_RPM2);

    tank.refuel();
    gauge_refuel(tank);
    gauge_fuel_qty(tank);
    
    
    
    step_Simulation(tInc);
    //prev_time=millis();
          
    
    gauge_Speed();
    rotary_loop();
    print_stats();
}

/*
 * void loop() {
  static uint32_t prevTime;
  static uint32_t currentTime;
  static uint32_t waitTime = 1000;
  currentTime = millis();
  if(currentTime - prevTime > waitTime) {
    prevTime = currentTime;
    youFuelFlowFunctionHere();
   }
}
---------------------
Thrust = {constant}*Throttle_Position.
Drag = C_D * V^2
V_new = V_old + F_x, where F_x = Thrust - Drag.
https://wright.nasa.gov/airplane/drageq.html
https://www.grc.nasa.gov/WWW/k-12/airplane/drageq.html
https://softschools.com/formulas/physics/drag_formula/470/
 */
