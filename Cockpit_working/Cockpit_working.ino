// Kids Flight Simulator
//
#include <Servo.h>
#include "tools.h"

#include "SevenSegmentTM1637.h"

int fuel_qty{0};

//Refuel 4-digit 7 segment Display
const byte PIN_CLK = 50;   // Green wire define CLK pin (any digital pin)
const byte PIN_DIO = 52;   // Yellow Wire define DIO pin (any digital pin)
SevenSegmentTM1637    refuel_display(PIN_CLK, PIN_DIO);

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
const int refuel_swtch{ 4 };
const int gears_swtch{ 5 };
const int f_pump_swtch{ 6 };
const int eng1_start{8};
const int eng2_start{9};
const int eng1_cutoff{10};
const int eng2_cutoff{11};
const uint8_t throttle1_knob{ A0 };
const uint8_t throttle2_knob{ A1 };

int pwr_sw_state{ 0 };
//int eng_sw_state{ 0 };
int rfl_sw_state{ 0 };
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

const int fuel_servo{ 30 };
const int rpm1_servo{ 31 };
const int rpm2_servo{ 32 };

const uint8_t speed_servo{ A2 };

Fuel_tank tank{10000,0};
int ref_amount{0};
Engine engine1(tank, eng1_start);
Engine engine2(tank, eng2_start);


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

void Fuel_tank::refuel(int x)
{
    qty += x; //lbs
    if (qty > cpcty) qty = cpcty;
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
            eng_rpm += 0.05;             // spools up
        }
        if (eng_rpm > 100.0 * flow) {
            eng_rpm -= 0.04;             // spools down
        }
    }

    if (!eng_ON || flow <= 0 || fuel_cut_off) {
        if(eng_rpm<0) eng_rpm = 0.0;
        else eng_rpm -= 0.04;
        //eng_rpm = 0.0;
    }

    return eng_rpm;
}

bool Engine::fuel_pump()
{
    
    if (f_pump_sw_state && pwr_sw_state) {
        digitalWrite(f_pump_LED, HIGH);
        return true;
    }
    else {
        digitalWrite(f_pump_LED, LOW);
        eng_ON=false;
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
        if (timer_second()) tank.consume(f_flow*40); // fuel is depleted from tank
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
    rfl_sw_state = digitalRead(refuel_swtch);
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
    }
    else {
        digitalWrite(pwr_LED, HIGH);
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
            digitalWrite(fuel_E_LED, HIGH);
            digitalWrite(fuel_L_LED, LOW);
        }

        else if (f.get_quantity() <= 10 * f.get_capacity() / 100) {
            digitalWrite(fuel_E_LED, LOW);
            digitalWrite(fuel_L_LED, HIGH);
        }

        else if (f.get_quantity() > 0) {
            digitalWrite(fuel_E_LED, LOW);
            digitalWrite(fuel_L_LED, LOW);
        }
        else {}
    }

    //if (rfl_sw_state == HIGH) f.refuel();
    //else {}
}
//--------------------------------------------------------

void gauge_RPM(Engine& e, Servo& servo)
{
    if (pwr_sw_state) {
        double angle = floatMap(e.get_rpm(), 0, 100, 0, 165);
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
    if (pwr_sw_state == HIGH && gears_sw_state == HIGH)
        digitalWrite(gears_LED, HIGH);
    else digitalWrite(gears_LED, LOW);
}

//--------------------------------------------------------

const double c_d = 0.001;
double drag{ 0 };
double speed(double v, Engine& e) {

    double thrust = 2.0 * e.get_rpm();
    drag = c_d * pow(v, 2);
    double v_new{ 0.0 };
    double v_old = v;
    double f_x = thrust - drag;
    v_new = v_old + f_x - 2;
    if (v_new < 0) v_new = 0.0;
    return v_new;

    // Drag = C_D * V^2
    // V_new = V_old + F_x, where F_x = Thrust - Drag.
}

void gauge_Speed()
{
    if (pwr_sw_state) {
        //double deg = speed_v * 180.0 / 500;
        double deg = floatMap(speed_v, 0, 500, 0, 179);
        //Serial.print("\t deg: ");
        //Serial.print(int(deg));
        servo_Speed.write(deg);
    }
    else servo_Speed.write(0);
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

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Refuel amnt: ");
    Serial.println(ref_amount);
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
      Serial.println("Button pressed!");
      tank.refuel(ref_amount);
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
  //refuel_display.print("FUEL");      // display INIT on the display
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
    
    //Serial.print("\t Throttle 1: ");
    //Serial.print(engine1.get_throttle());
    //Serial.print("\t Throttle 2: ");
    //Serial.print(engine2.get_throttle());
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
    
    servo_RPM1.attach(rpm1_servo);
    servo_RPM2.attach(rpm2_servo);
    servo_Fuel.attach(fuel_servo);
    servo_Speed.attach(speed_servo);
    engine2.flip_throttle(true);

    rotary_setup();
    refuel_DSP_setup();
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
    
    gauge_eng(engine1);
    gauge_eng(engine2);
 
    
    gauge_RPM(engine1, servo_RPM1);
    gauge_RPM(engine2, servo_RPM2);
    
    gauge_refuel(tank);
    gauge_fuel_qty(tank);
    gauge_Speed();
    speed_v = speed(speed_v, engine1);
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
