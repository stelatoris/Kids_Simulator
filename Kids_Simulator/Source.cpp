// Kids Flight Simulator
//
#include <Servo.h>

Servo servo_RPM;
Servo servo_Fuel;

const int power_swtch{ 2 };
const int eng_swtch{ 3 };
const int refuel_swtch{ 6 };
int pwr_sw_state{ 0 };
int eng_sw_state{ 0 };
int rfl_sw_state{ 0 };
int f_pump_sw_state{ 0 };

int gears_sw_state{ 0 };
const int pwr_LED{ 4 };
const int eng_LED{ 5 };
const int fuel_L_LED{ 7 };
const int gears_LED{ 11 };
const int gears_swtch{ 12 };
const int f_pump_LED{ 8 };
const int f_pump_swtch{ 13 };

int throttle_value = 0;


bool engine_on = false;

//---------------------------------

unsigned long time_begin = millis();
unsigned long seconds{ 0 };

bool timer_second()
// return true every 1000 millis
{

    if (1000 <= millis() - time_begin &&
        millis() - time_begin <= 1100) {
        ++seconds;
        Serial.println(seconds);
        time_begin = millis();
        return true;
    }
    else { return false; }
}

//---------------------------------

void check_inputs()
{
    pwr_sw_state = digitalRead(power_swtch);
    eng_sw_state = digitalRead(eng_swtch);
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
        digitalWrite(fuel_L_LED, LOW);
        digitalWrite(f_pump_LED, LOW);
    }
    else {
        digitalWrite(pwr_LED, HIGH);
    }
}
//---------------------------------

double fuel_qty{ 0.0 };
const double tank_capacity{ 10000.0 };

void refuel()
{
    fuel_qty = 5000; //lbs
}
//---------------------------------

bool fuel_pump()
{
    if (f_pump_sw_state && pwr_sw_state) {
        digitalWrite(f_pump_LED, HIGH);
        return true;
    }
    else {
        digitalWrite(f_pump_LED, LOW);
        return false;
    }
}

double throttle()
{
    return int(throttle_value * 100.0 / 1023.0);
}

double fuel_flow()

{
    double gps = 1.0;  // gives gallons per second at 100% throttle
    if (0 < fuel_qty && eng_sw_state && fuel_pump()) {
        double flow = throttle() * gps / 100; // gallons
        if (timer_second()) fuel_qty -= (flow * 1000); // fuel is depleted from tank
        if (fuel_qty <= 0) fuel_qty = 0;
        return flow;
    }
    else { return 0.0; }
}
//--------------------------------------------------------

double rpm{ 0.0 };

void engine()
{
    if (eng_sw_state && fuel_flow() > 0) rpm = 100 * fuel_flow(); // 100% RPM at fuel flow of 1 gallon per second

    if (rpm > 0 && eng_sw_state) engine_on = true;
    else engine_on == false;

    if (!eng_sw_state || fuel_flow() <= 0 || fuel_qty <= 0) {
        rpm = 0.0;
        engine_on = false;
    }
}

//----------
void gauge_RPM()
{
    if (pwr_sw_state) {
        int deg = rpm * 180 / 100;
        servo_RPM.write(deg);
    }
    else servo_RPM.write(0);
}

//--------------------------------------------------------
void gauge_fuel_qty()
{
    if (pwr_sw_state == HIGH) {
        int lvl = fuel_qty * 180 / tank_capacity;
        servo_Fuel.write(180 - lvl);
    }
    else servo_Fuel.write(180);

}

//----------
void gauge_refuel()
{
    if (pwr_sw_state == HIGH && fuel_qty == 0) {
        digitalWrite(fuel_L_LED, HIGH);
    }
    else if (pwr_sw_state == HIGH && fuel_qty > 0) {
        digitalWrite(fuel_L_LED, LOW);
    }
    else {
        digitalWrite(fuel_L_LED, LOW);
    }

    if (rfl_sw_state == HIGH) refuel();
    else {}

    gauge_fuel_qty();

}

//----------

void gauge_eng()
{
    if (engine_on == false) {
        digitalWrite(eng_LED, LOW);
    }
    else if (pwr_sw_state && engine_on == true) {
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

void print_stats()
{
    Serial.print("\t Time(millis): ");
    Serial.print(millis());
    //Serial.print("\t Time(s): ");
    //Serial.print(seconds);
    Serial.print("\t Fuel Amount: ");
    Serial.print(fuel_qty);
    Serial.print("\t Fuel flow: ");
    Serial.print(fuel_flow());
    Serial.print("\t Throttle: ");
    Serial.print(throttle());
    Serial.print("\t RPM: ");
    Serial.print(rpm);
    Serial.println();
}
void setup()
{
    Serial.begin(9600);
    pinMode(power_swtch, INPUT);
    pinMode(pwr_LED, OUTPUT);
    pinMode(eng_LED, OUTPUT);
    pinMode(fuel_L_LED, OUTPUT);
    pinMode(gears_LED, OUTPUT);
    pinMode(gears_swtch, INPUT);
    pinMode(f_pump_swtch, INPUT);
    pinMode(f_pump_LED, OUTPUT);
    pinMode(A0, INPUT);
    servo_RPM.attach(9);
    servo_Fuel.attach(10);
}

void loop()
{
    check_inputs();
    timer_second();
    throttle_value = analogRead(A0);
    fuel_pump();
    engine();
    gears();
    gauge_pwr();
    gauge_eng();
    gauge_RPM();
    gauge_refuel();
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