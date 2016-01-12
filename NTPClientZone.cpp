/**
 * The MIT License (MIT)
 * Copyright (c) 2015 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "NTPClientZone.h"

NTPClientZone::NTPClientZone() {}

NTPClientZone::NTPClientZone(int timeOffset) {
  this->_timeOffset     = timeOffset;
}

NTPClientZone::NTPClientZone(const char* poolServerName) {
  this->_poolServerName = poolServerName;
}

NTPClientZone::NTPClientZone(const char* poolServerName, int timeOffset) {
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
}

NTPClientZone::NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval) {
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
  this->_updateInterval = updateInterval;
}

NTPClientZone::NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval, TimeChangeRule myDST, TimeChangeRule mySTD) {
  this->setZones(myDST, mySTD);
  this->_timeOffset     = timeOffset;
  this->_poolServerName = poolServerName;
  this->_updateInterval = updateInterval;
}

// NTPClientZone::NTPClientZone(const char* poolServerName, int timeOffset, int updateInterval, int address) {
//   readRules(address);
//   this->_timeOffset     = timeOffset;
//   this->_poolServerName = poolServerName;
//   this->_updateInterval = updateInterval;
// }

void NTPClientZone::setTimeOffset(int timeOffset) {
	this->_timeOffset		= timeOffset;
}

void NTPClientZone::forceUpdate() {
  #ifdef DEBUG_NTPClientZone
    Serial.println("Update from NTP Server");
  #endif

  IPAddress address;
  WiFi.hostByName(this->_poolServerName, address);

  this->sendNTPPacket(address);

  // Wait till data is there or timeout...
  byte timeout = 0;
  int cb = 0;
  do {
    delay ( 10 );
    cb = this->_udp.parsePacket();
    if (timeout > 100) return; // timeout after 1000 ms
    timeout++;
  } while (cb == 0);

  this->_lastUpdate = millis() - (10 * (timeout + 1)); // Account for delay in reading the time

  this->_udp.read(this->_packetBuffer, NTP_PACKET_SIZE);

  unsigned long highWord = word(this->_packetBuffer[40], this->_packetBuffer[41]);
  unsigned long lowWord = word(this->_packetBuffer[42], this->_packetBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  unsigned long secsSince1900 = highWord << 16 | lowWord;

  this->_currentEpoc = secsSince1900 - SEVENZYYEARS;
  // this->_timeOffset += locIsDST(toLocal(_currentEpoc)) * 60 * 60;
}

void NTPClientZone::update() {
  if ((millis() - this->_lastUpdate >= this->_updateInterval)     // Update after _updateInterval
    || this->_lastUpdate == 0) {                                // Update if there was no update yet.
    if (this->_lastUpdate == 0) this->_udp.begin(this->_port);  // Start _udp if there was no update yet.
    this->forceUpdate();
  }
}

unsigned long NTPClientZone::getRawTime() {
  return ((this->_timeOffset + locIsDST(toLocal(_currentEpoc))) * 60 * 60) + // User offset
         this->_currentEpoc + // Epoc returned by the NTP server
         ((millis() - this->_lastUpdate) / 1000); // Time since last update
}

unsigned long NTPClientZone::getUtcTime() {
  return _currentEpoc + ((millis() - this->_lastUpdate) / 1000);
}

String NTPClientZone::getDay() {
  return dayStrings[(getRawTime() / 86400L) % 7];
}
String NTPClientZone::getHours() {
  return String((this->getRawTime()  % 86400L) / 3600);
}
String NTPClientZone::getMinutes() {
  return String((this->getRawTime() % 3600) / 60);
}

String NTPClientZone::getSeconds() {
  return String(this->getRawTime() % 60);
}

float NTPClientZone::getDayNum() {
  return (getRawTime() / 86400L) % 7;
}
float NTPClientZone::getHoursNum() {
  return (this->getRawTime()  % 86400L) / 3600;
}
float NTPClientZone::getMinutesNum() {
  return (this->getRawTime() % 3600) / 60;
}

float NTPClientZone::getSecondsNum() {
  return this->getRawTime() % 60;
}

String NTPClientZone::getFormattedTime() {
  unsigned long rawTime = this->getRawTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return hoursStr + ":" + minuteStr + ":" + secondStr;
}

void NTPClientZone::sendNTPPacket(IPAddress ip) {
  // set all bytes in the buffer to 0
  memset(this->_packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  this->_packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  this->_packetBuffer[1] = 0;     // Stratum, or type of clock
  this->_packetBuffer[2] = 6;     // Polling Interval
  this->_packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  this->_packetBuffer[12]  = 49;
  this->_packetBuffer[13]  = 0x4E;
  this->_packetBuffer[14]  = 49;
  this->_packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  this->_udp.beginPacket(ip, 123); //NTP requests are to port 123
  this->_udp.write(this->_packetBuffer, NTP_PACKET_SIZE);
  this->_udp.endPacket();
}

void NTPClientZone::setZones(TimeChangeRule dstStart, TimeChangeRule stdStart) {
    _dst = dstStart;
    _std = stdStart;
}

/*----------------------------------------------------------------------*
 * Convert the given UTC time to local time, standard or                *
 * daylight time, as appropriate.                                       *
 *----------------------------------------------------------------------*/
time_t NTPClientZone::toLocal(time_t utc)
{
    //recalculate the time change points if needed
    if (year(utc) != year(_dstUTC)) calcTimeChanges(year(utc));

    if (utcIsDST(utc))
        return utc + _dst.offset * SECS_PER_MIN;
    else
        return utc + _std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Convert the given UTC time to local time, standard or                *
 * daylight time, as appropriate, and return a pointer to the time      *
 * change rule used to do the conversion. The caller must take care     *
 * not to alter this rule.                                              *
 *----------------------------------------------------------------------*/
time_t NTPClientZone::toLocal(time_t utc, TimeChangeRule **tcr)
{
    //recalculate the time change points if needed
    if (year(utc) != year(_dstUTC)) calcTimeChanges(year(utc));

    if (utcIsDST(utc)) {
        *tcr = &_dst;
        return utc + _dst.offset * SECS_PER_MIN;
    }
    else {
        *tcr = &_std;
        return utc + _std.offset * SECS_PER_MIN;
    }
}

/*----------------------------------------------------------------------*
 * Convert the given local time to UTC time.                            *
 *                                                                      *
 * WARNING:                                                             *
 * This function is provided for completeness, but should seldom be     *
 * needed and should be used sparingly and carefully.                   *
 *                                                                      *
 * Ambiguous situations occur after the Standard-to-DST and the         *
 * DST-to-Standard time transitions. When changing to DST, there is     *
 * one hour of local time that does not exist, since the clock moves    *
 * forward one hour. Similarly, when changing to standard time, there   *
 * is one hour of local times that occur twice since the clock moves    *
 * back one hour.                                                       *
 *                                                                      *
 * This function does not test whether it is passed an erroneous time   *
 * value during the Local -> DST transition that does not exist.        *
 * If passed such a time, an incorrect UTC time value will be returned. *
 *                                                                      *
 * If passed a local time value during the DST -> Local transition      *
 * that occurs twice, it will be treated as the earlier time, i.e.      *
 * the time that occurs before the transistion.                         *
 *                                                                      *
 * Calling this function with local times during a transition interval  *
 * should be avoided!                                                   *
 *----------------------------------------------------------------------*/
time_t NTPClientZone::toUTC(time_t local)
{
    //recalculate the time change points if needed
    if (year(local) != year(_dstLoc)) calcTimeChanges(year(local));

    if (locIsDST(local))
        return local - _dst.offset * SECS_PER_MIN;
    else
        return local - _std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Determine whether the given UTC time_t is within the DST interval    *
 * or the Standard time interval.                                       *
 *----------------------------------------------------------------------*/
boolean NTPClientZone::utcIsDST(time_t utc)
{
    //recalculate the time change points if needed
    if (year(utc) != year(_dstUTC)) calcTimeChanges(year(utc));

    if (_stdUTC > _dstUTC)    //northern hemisphere
        return (utc >= _dstUTC && utc < _stdUTC);
    else                      //southern hemisphere
        return !(utc >= _stdUTC && utc < _dstUTC);
}

/*----------------------------------------------------------------------*
 * Determine whether the given Local time_t is within the DST interval  *
 * or the Standard time interval.                                       *
 *----------------------------------------------------------------------*/
boolean NTPClientZone::locIsDST(time_t local)
{
    //recalculate the time change points if needed
    if (year(local) != year(_dstLoc)) calcTimeChanges(year(local));

    if (_stdLoc > _dstLoc)    //northern hemisphere
        return (local >= _dstLoc && local < _stdLoc);
    else                      //southern hemisphere
        return !(local >= _stdLoc && local < _dstLoc);
}

/*----------------------------------------------------------------------*
 * Calculate the DST and standard time change points for the given      *
 * given year as local and UTC time_t values.                           *
 *----------------------------------------------------------------------*/
void NTPClientZone::calcTimeChanges(int yr)
{
    _dstLoc = toTime_t(_dst, yr);
    _stdLoc = toTime_t(_std, yr);
    _dstUTC = _dstLoc - _std.offset * SECS_PER_MIN;
    _stdUTC = _stdLoc - _dst.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
 * Convert the given DST change rule to a time_t value                  *
 * for the given year.                                                  *
 *----------------------------------------------------------------------*/
time_t NTPClientZone::toTime_t(TimeChangeRule r, int yr)
{
    tmElements_t tm;
    time_t t;
    uint8_t m, w;            //temp copies of r.month and r.week

    m = r.month;
    w = r.week;
    if (w == 0) {            //Last week = 0
        if (++m > 12) {      //for "Last", go to the next month
            m = 1;
            yr++;
        }
        w = 1;               //and treat as first week of next month, subtract 7 days later
    }

    tm.Hour = r.hour;
    tm.Minute = 0;
    tm.Second = 0;
    tm.Day = 1;
    tm.Month = m;
    tm.Year = yr - 1970;
    t = makeTime(tm);        //first day of the month, or first day of next month for "Last" rules
    t += (7 * (w - 1) + (r.dow - weekday(t) + 7) % 7) * SECS_PER_DAY;
    if (r.week == 0) t -= 7 * SECS_PER_DAY;    //back up a week if this is a "Last" rule
    return t;
}


/*----------------------------------------------------------------------*
 * Read the daylight and standard time rules from EEPROM at       *
 * the given address.                                                   *
 *----------------------------------------------------------------------*/
// void NTPClientZone::readRules(int address)
// {
//     EEPROM.begin(512);
//     EEPROM.eeprom_read_block((void *) &_dst, (void *) address, sizeof(_dst));
//     address += sizeof(_dst);
//     EEPROM.eeprom_read_block((void *) &_std, (void *) address, sizeof(_std));
// }

/*----------------------------------------------------------------------*
 * Write the daylight and standard time rules to EEPROM at        *
 * the given address.                                                   *
 *----------------------------------------------------------------------*/
// void NTPClientZone::writeRules(int address)
// {
    // EEPROM.begin(512);
    // eeprom_write_block((void *) &_dst, (void *) address, sizeof(_dst));
    // address += sizeof(_dst);
    // eeprom_write_block((void *) &_std, (void *) address, sizeof(_std));
// }


