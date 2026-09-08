#include "exploration_planner/sensor_freshness.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using exploration::SensorFreshness;

void check(bool value, const char* message)
{
    if (!value) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

int main()
{
    SensorFreshness sensor;
    check(!sensor.fresh(100.0, 0.3), "unseen sensor is not fresh");
    check(sensor.observe(10000000000LL, 100.0), "sensor uptime stamp is accepted");
    check(sensor.fresh(100.1, 0.3), "receipt age ignores sensor clock offset");
    check(!sensor.observe(10000000000LL, 100.2), "a repeated frame is not new data");
    check(!sensor.fresh(100.4, 0.3), "repeated stamps cannot keep the watchdog alive");
    check(sensor.observe(10100000000LL, 100.5), "new frames recover after a gap");
    check(std::fabs(sensor.source_interval() - 0.1) < 1e-9, "velocity uses source intervals");
    check(!sensor.observe(1000000000LL, 100.6), "backward stamp reinitializes the source");
    check(!sensor.fresh(100.6, 0.3), "source restart requires another progressing frame");
    check(sensor.observe(1100000000LL, 100.7), "progress resumes after source restart");
    check(sensor.source_interval() == 0.0, "restart does not create a velocity impulse");
    check(sensor.fresh(100.8, 0.3), "restart does not permanently lock out valid frames");
    check(!sensor.fresh(100.6, 0.3), "backward local query is not treated as fresh");
    check(!sensor.observe(-1, 100.9), "invalid source stamp is rejected");
    check(!sensor.observe(1200000000LL, std::numeric_limits<double>::quiet_NaN()),
          "invalid arrival time is rejected");
    check(!sensor.fresh(101.1, 0.3), "invalid observations do not extend freshness");

    SensorFreshness epoch;
    check(epoch.observe(1788873176000000000LL, 10.0), "epoch source stamp is accepted");
    check(epoch.fresh(10.2, 0.3), "source clock ahead of local clock is supported");
    SensorFreshness zero;
    check(zero.observe(0, 1.0), "zero is a valid initial simulation time");
    check(!zero.observe(0, 2.0) && !zero.fresh(2.0, 0.3), "frozen zero time expires");
    check(zero.observe(20000000, 2.1) && zero.fresh(2.2, 0.3), "simulation clock can start later");
    std::cout << "sensor_freshness: all checks passed\n";
}
