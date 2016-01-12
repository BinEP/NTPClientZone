#pragma once

#include "Arduino.h"

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// #include <Timezone.h>
#include <Time.h>

#define SEVENZYYEARS 2208988800UL
#define NTP_PACKET_SIZE 48

//convenient constants for dstRules
enum week_t {Last, First, Second, Third, Fourth}; 
enum dow_t {Sun=1, Mon, Tue, Wed, Thu, Fri, Sat};
enum month_t {Jan=1, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};

//structure to describe rules for when daylight/summer time begins,
//or when standard time begins.
struct TimeChangeRule
{
    char abbrev[6];    //five chars max
    uint8_t week;      //First, Second, Third, Fourth, or Last week of the month
    uint8_t dow;       //day of week, 1=Sun, 2=Mon, ... 7=Sat
    uint8_t month;     //1=Jan, 2=Feb, ... 12=Dec
    uint8_t hour;      //0-23
    int offset;        //offset from UTC in minutes
};

class NTPClientZone {
  private:
    WiFiUDP       _udp;

    TimeChangeRule myDST;    //Daylight time = UTC - 4 hours
    TimeChangeRule mySTD;     //Standard time = UTC - 5 hours
    // Timezone _myTZ;

    const char*   _poolServerName = "time.nist.gov"; // Default time server
    int           _port           = 1337;
    int           _timeOffset;

    unsigned int  _updateInterval = 60000;  // In ms

    unsigned long _currentEpoc;             // In s
    unsigned long _lastUpdate     = 0;      // In ms

    byte          _packetBuffer[NTP_PACKET_SIZE];
    String        dayStrings[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

    void          sendNTPPacket(IPAddress _timeServerIP);

    void calcTimeChanges(int yr);
    time_t toTime_t(TimeChangeRule r, int yr);
    TimeChangeRule _dst;    //rule for start of dst or summer time for any year
    TimeChangeRule _std;    //rule for start of standard time for any year
    time_t _dstUTC;         //dst start for given/current year, given in UTC
    time_t _stdUTC;         //std time start for given/current year, given in UTC
    time_t _dstLoc;         //dst start for given/current year, given in local time
    time_t _stdLoc;         //std time start for given/current year, given in local time

  public:
    NTPClientZone();
    NTPClientZone(int timeOffset);
    NTPClientZone(const char* poolServerName);
    NTPClientZone(const char* poolServerName, int timeOffset);
    NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval);
    NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval, TimeChangeRule myDST, TimeChangeRule mySTD);
    // NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval, int address);
    
    /**
     * This should be called in the main loop of your application. By default an update from the NTP Server is only
     * made every 60 seconds. This can be configured in the NTPClientZone
    constructor.
     */
    void update();

    /**
     * This will force the update from the NTP Server.
     */
    void forceUpdate();

    String getDay();
    String getHours();
    String getMinutes();
    String getSeconds();

    float getDayNum();
    float getHoursNum();
    float getMinutesNum();
    float getSecondsNum();

    void setTimeOffset(int timeOffset);

    /**
     * @return time formatted like `hh:mm:ss`
     */
    String getFormattedTime();

    /**
     * @return time as raw seconds
     */
    unsigned long getRawTime();

    /**
     * @return time of utc
     */
     unsigned long getUtcTime();

    void setZones(TimeChangeRule dstStart, TimeChangeRule stdStart);
    time_t toLocal(time_t utc);
    time_t toLocal(time_t utc, TimeChangeRule **tcr);
    time_t toUTC(time_t local);
    boolean utcIsDST(time_t utc);
    boolean locIsDST(time_t local);
    // void readRules(int address);
    // void writeRules(int address);
};
