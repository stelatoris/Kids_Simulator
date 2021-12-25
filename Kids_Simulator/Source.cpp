// Kids Flight Simulator
//
#include <Servo.h>

Servo servo_RPM;
Servo servo_Fuel;
Servo servo_Speed;

const int power_swtch{ 2 };
const int eng_swtch{ 3 };
const int refuel_swtch{ 40 };
const int gears_swtch{ 5 };
const int f_pump_swtch{ 6 };
const uint8_t throttle_knob{ A0 };

int pwr_sw_state{ 0 };
int eng_sw_state{ 0 };
int rfl_sw_state{ 0 };
int f_pump_sw_state{ 0 };
int gears_sw_state{ 0 };

const int pwr_LED{ 22 };
const int eng_LED{ 23 };
const int fuel_L_LED{ 24 };
const int f_pump_LED{ 25 };
const int gears_LED{ 26 };

const int fuel_servo{ 30 };
const int rpm_servo{ 31 };
const uint8_t speed_servo{ A1 };

double throttle_value = 0;


bool engine_on = false;
double speed_v{ 0 };

double floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
    fuel_qty = 10000; //lbs
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
    double angle = floatMap(throttle_value, 0, 1023, 0, 100);
    return angle;
    //return throttle_value * 100.0 / 1023.0;
}

double fuel_flow()

{
    double gps = 1.0;  // gives gallons per second at 100% throttle
    if (0 < fuel_qty && eng_sw_state && fuel_pump()) {
        double flow = throttle() * gps / 100.0; // gallons
        if (timer_second()) fuel_qty -= (flow * 100.0); // fuel is depleted from tank
        if (fuel_qty <= 0) fuel_qty = 0;
        return flow;
    }
    else { return 0.0; }
}

//----------
void gauge_fuel_qty()
{
    if (pwr_sw_state == HIGH) {
        double lvl = fuel_qty * 180 / tank_capacity;
        servo_Fuel.write(180 - int(lvl));
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



}
//--------------------------------------------------------

double rpm{ 0.0 };

void engine()
{
    if (eng_sw_state && fuel_flow() > 0) rpm = 100.0 * fuel_flow(); // 100% RPM at fuel flow of 1 gallon per second

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
        double deg = rpm * 180.0 / 100;
        servo_RPM.write(int(deg));
    }
    else servo_RPM.write(0.0);
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

const double c_d = 0.001;
double drag{ 0 };
double speed(double v) {

    double thrust = 2.0 * rpm;
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
        double deg = speed_v * 180.0 / 500;
        Serial.print("\t deg: ");
        Serial.print(int(deg));
        servo_Speed.write(deg);
    }
    else servo_Speed.write(0);
}
//--------------------------------------------------------

void print_stats()
{
    ///Serial.print("\t Time(millis): ");
    //Serial.print(millis());
    Serial.print("\t Time(s): ");
    Serial.print(seconds);
    Serial.print("\t Fuel Amount: ");
    Serial.print(fuel_qty);
    Serial.print("\t Fuel flow: ");
    Serial.print(fuel_flow());
    Serial.print("\t Throttle: ");
    Serial.print(throttle());
    Serial.print("\t RPM: ");
    Serial.print(rpm);
    Serial.print("\t Speed: ");
    Serial.print(speed_v);
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
    pinMode(throttle_knob, INPUT);
    pinMode(speed_servo, OUTPUT);
    pinMode(rpm_servo, OUTPUT);
    pinMode(fuel_servo, OUTPUT);
    servo_RPM.attach(rpm_servo);
    servo_Fuel.attach(fuel_servo);
    servo_Speed.attach(speed_servo);
}

void loop()
{
    check_inputs();
    timer_second();
    throttle_value = analogRead(throttle_knob);
    fuel_pump();
    engine();
    gears();
    gauge_pwr();
    gauge_eng();
    gauge_RPM();
    gauge_refuel();
    gauge_fuel_qty();
    gauge_Speed();
    speed_v = speed(speed_v);
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