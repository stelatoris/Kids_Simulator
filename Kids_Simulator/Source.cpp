// Kids Flight Simulator
//
#include <Servo.h>

Servo servo;

const int power_swtch{ 2 };
const int eng_swtch{ 3 };
const int refuel_swtch{ 6 };
int pwr_sw_state{ 0 };
int eng_sw_state{ 0 };
int rfl_sw_state{ 0 };
const int pwr_LED{ 4 };
const int eng_LED{ 5 };
const int fuel_L_LED{ 7 };
const int fuel_H_LED{ 8 };

int throttle_value = 0;


bool engine_on = false;

//---------------------------------

unsigned long time_begin = millis();
unsigned long seconds{ 0 };


bool timer_second()
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
}

//---------------------------------
void gauge_pwr()
{
    if (pwr_sw_state == LOW) {
        digitalWrite(pwr_LED, LOW);
        digitalWrite(eng_LED, LOW);
        digitalWrite(fuel_L_LED, LOW);
        digitalWrite(fuel_H_LED, LOW);
    }
    else {
        digitalWrite(pwr_LED, HIGH);
    }
}
//---------------------------------

double fuel_qty{ 0.0 };

void refuel()
{
    fuel_qty = 10000; //lbs
}
//---------------------------------

double throttle()
{
    return throttle_value * 100.0 / 1023.0;
}

double fuel_flow()
{
    double gps = 1.0;  // gives gallons per second at 100% throttle
    if (0 < fuel_qty) {
        double flow = throttle() * gps / 100; // gallons
        if (timer_second()) fuel_qty -= (flow * 1000);
        if (fuel_qty <= 0) fuel_qty = 0;
        return flow;
    }
    else { return 0.0; }

}
//---------------------------------

double rpm{ 0.0 };

void engine()
{
    if (eng_sw_state == 1 && fuel_flow() > 0) rpm = 100 * fuel_flow(); // 100% RPM at fuel flow of 1 gallon per second

    if (rpm > 0 && eng_sw_state == 1) engine_on = true;
    else engine_on == false;

    if (eng_sw_state == 0 || fuel_flow() <= 0 || fuel_qty <= 0) {
        rpm = 0.0;
        engine_on = false;
    }
}

//---------------------------------
void gauge_RPM()
{
    int deg = rpm * 180 / 100;
    servo.write(deg);
}

//---------------------------------
void gauge_refuel()
{
    if (pwr_sw_state && fuel_qty == 0) {
        digitalWrite(fuel_L_LED, HIGH);
        digitalWrite(fuel_H_LED, LOW);
    }
    else if (fuel_qty > 0) {
        digitalWrite(fuel_L_LED, LOW);
        digitalWrite(fuel_H_LED, HIGH);
    }
    else {
        digitalWrite(fuel_L_LED, LOW);
        digitalWrite(fuel_H_LED, LOW);
    }

    if (rfl_sw_state == HIGH) refuel();
    else {}

}
//-----------------------------------------------------------

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
//---------------------------------

void print_stats()
{
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
    Serial.println();
}
void setup()
{
    Serial.begin(9600);
    pinMode(power_swtch, INPUT);
    pinMode(pwr_LED, OUTPUT);
    pinMode(eng_LED, OUTPUT);
    pinMode(A0, INPUT);
    servo.attach(9);
}

void loop()
{
    check_inputs();
    throttle_value = analogRead(A0);
    engine();
    gauge_pwr();
    gauge_eng();
    gauge_RPM();
    gauge_refuel();
    print_stats();
    timer_second();

}