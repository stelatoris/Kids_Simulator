#pragma once

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

//---------------------------------

double floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct Fuel_tank {

    Fuel_tank(double cap, double quant) : cpcty{ cap }, qty{ quant } {}
    Fuel_tank() : cpcty{ 10000 }, qty{ 5000 } {}

    double get_quantity() const { return qty; }
    void set_quantity(double q) { qty = q; }
    void consume(double q) { qty -= q; }
    void set_capacity(double c) { cpcty = c; }
    double get_capacity() const { return cpcty; }

    void refuel();
private:
    double cpcty;
    double qty;
};

//---------------------------------

struct Engine {
    Engine(Fuel_tank& f) : tank{ f } {}

    double rpm();
    double fuel_flow();
    double get_throttle();
    bool fuel_pump();
    bool engineON() { return eng_ON; }
    void set_throttle(double t) { throttle = t; }

    //int eng_num;

private:
    Fuel_tank& tank;
    double throttle;
    double eng_rpm;
    double gps = 1.0; // gives gallons per second at 100% throttle
    double f_flow;
    bool eng_ON;
};
