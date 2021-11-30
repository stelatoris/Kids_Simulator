// C++ code
//

#include <iostream>
#include <chrono>
using namespace std;
using namespace chrono;

const int HIGH = 1;
const int LOW = 0;

int digitalRead(int p)
{
    return 1;
}

void digitalWrite(int p, bool s)
{
    if (s) std::cout << p << "LED is ON" << '\n';
    else std::cout << p << "LED is OFF" << '\n';
    
}

//----------------------------------------------------------------

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

bool engine_on = false;

//---------------------------------

auto time_begin = high_resolution_clock::now();
int seconds{ 0 };



bool timer_second()
{
    return true;
}

//---------------------------------

void check_inputs()
{
    pwr_sw_state = digitalRead(power_swtch);
    eng_sw_state = digitalRead(eng_swtch);
    rfl_sw_state = 0;
}

//---------------------------------
void gauge_pwr()
{
    if (pwr_sw_state == LOW) {
        digitalWrite(pwr_LED, LOW);
        digitalWrite(eng_LED, LOW);
    }
    else {
        digitalWrite(pwr_LED, HIGH);
    }
}

//---------------------------------

unsigned int fuel_qty{ 10000 };

void refuel()
{
    fuel_qty = 10000; //lbs
}

//---------------------------------

double throttle{ 50.0 };  // will read from sensor/potentiometer

double fuel_flow()
{
    double gps = 1.0;  // gives gallons per second at 100% throttle
    if (0 < fuel_qty && pwr_sw_state) {
        double flow = throttle * gps / 100; // gallons
        if (timer_second()) fuel_qty -= flow;
        return flow;
    }
    else { return 0.0; }

}
//---------------------------------

int rpm{ 0 };

void engine()
{
    rpm = 100 * fuel_flow(); // 100% RPM at fuel flow of 1 gallon per second
    if (fuel_flow() > 0 && 0 < rpm && eng_sw_state) { engine_on = true; }
    else { engine_on = false; }
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
    if (!engine_on) {
        digitalWrite(eng_LED, LOW);
    }
    else if (pwr_sw_state == HIGH && engine_on) {
        digitalWrite(eng_LED, HIGH);
    }
    else {}
}

//---------------------------------

void print_stats()
{
    std::cout << "\t Time(s): ";
    std::cout << "\t Fuel Amount: " << fuel_qty;
    std::cout << "\t Fuel flow: " << fuel_flow;
    std::cout << "\t Throttle: " << throttle;
    std::cout << "\t RPM: " << rpm;
    std::cout << '\n';
}

int main()
{
    milliseconds foo(1);
    int looper = 1;
    while (looper==1) {
        check_inputs();
        engine();
        gauge_pwr();
        gauge_eng();
        gauge_refuel();
        print_stats();
        //timer_second();
    }
}