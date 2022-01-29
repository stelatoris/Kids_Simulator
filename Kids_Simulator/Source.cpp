// Kids Flight Simulator
//
#include <Servo.h>
#include "tools.h"

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
const int fuel_E_LED{ 24 };
const int fuel_L_LED{ 27 };
const int f_pump_LED{ 25 };
const int gears_LED{ 26 };

const int fuel_servo{ 30 };
const int rpm_servo{ 31 };
const uint8_t speed_servo{ A1 };

Fuel_tank tank;
Engine engine1(tank);

double speed_v{ 0 };

unsigned long prev_time = millis();
unsigned long seconds{ 0 };

bool timer_second()
// return true every 1000 millis
{

    if (1000 <= millis() - prev_time) {
        prev_time = millis();
        ++seconds;
        Serial.println(seconds);

        return true;
    }
    else { return false; }
}

//----------------------------------------------------------------------------

void Fuel_tank::refuel()
{
    qty += 1000; //lbs
    if (qty > cpcty) qty = cpcty;
}

double Engine::get_throttle()
{
    double angle = floatMap(throttle, 0, 1023, 0, 100);
    return angle;
    //return throttle_value * 100.0 / 1023.0;
}

double Engine::rpm()
{
    double flow = fuel_flow();
    if (flow > 0.0) {               //spools RPM
        if (eng_rpm < 100.0 * flow) {
            eng_rpm += 1.0;             // spools up
        }
        if (eng_rpm > 100.0 * flow) {
            eng_rpm -= 1.0;             // spools down
        }
    }

    if (eng_rpm > 0 && eng_sw_state) eng_ON = true;
    else eng_ON == false;

    if (!eng_sw_state || flow <= 0) {
        eng_rpm = 0.0;
        eng_ON = false;
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
        return false;
    }
}

double Engine::fuel_flow()
{
    if (0 < tank.get_quantity() && eng_sw_state && fuel_pump()) {
        f_flow = get_throttle() * gps / 100.0; // gallons
        if (timer_second()) tank.consume(f_flow * 100); // fuel is depleted from tank
        if (tank.get_quantity() <= 0) tank.set_quantity(0.0);
    }
    else { f_flow = 0.0; }
    return f_flow;
}

//--------------------------------------------------------------------------------------------

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
        servo_Fuel.write(180 - int(lvl));
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

    if (rfl_sw_state == HIGH) f.refuel();
    else {}
}
//--------------------------------------------------------

void gauge_RPM(Engine& e)
{
    if (pwr_sw_state) {
        double deg = e.rpm() * 180.0 / 100;
        servo_RPM.write(int(deg));
    }
    else servo_RPM.write(0.0);
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

    double thrust = 2.0 * e.rpm();
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
    Serial.print(tank.get_quantity());
    Serial.print("\t Fuel flow: ");
    Serial.print(engine1.fuel_flow());
    Serial.print("\t Throttle: ");
    Serial.print(engine1.get_throttle());
    Serial.print("\t RPM: ");
    Serial.print(engine1.rpm());
    Serial.print("\t Speed: ");
    Serial.print(speed_v);
    Serial.println();
}

void engine_status(Engine& e)
{
    e.fuel_pump();
    e.rpm();
}

void setup()
{
    Serial.begin(9600);
    pinMode(power_swtch, INPUT);
    pinMode(pwr_LED, OUTPUT);
    pinMode(eng_LED, OUTPUT);
    pinMode(fuel_E_LED, OUTPUT);
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
    engine1.set_throttle(analogRead(throttle_knob));

    engine_status(engine1);
    gears();
    gauge_pwr();
    gauge_eng(engine1);
    gauge_RPM(engine1);
    gauge_refuel(tank);
    gauge_fuel_qty(tank);
    gauge_Speed();
    speed_v = speed(speed_v, engine1);
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