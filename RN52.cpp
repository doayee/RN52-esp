/*
	RN52.cpp
	Adapted from the espsoftwareserial library by plerup.
	See https://github.com/plerup/espsoftwareserial
	With added functionality to interface with the RN52 from Microchip.
	Written by Thomas Cousins and Thomas McQueen for https://doayee.co.uk

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <RN52.h>

RN52 *RN52::active_object = NULL;
short RN52::IOMask = 0x0004;
short RN52::IOProtect = 0x3C64;
short RN52::IOStateMask = 0x0094;
short RN52::IOStateProtect = 0x3CF4;
short RN52::IO;
short RN52::IOState;
volatile bool  RN52::_trackChanged = false;

#ifndef ESP32
#ifndef RN52_MAX_INSTS
#define RN52_MAX_INSTS 1
#endif

// As the ESP8266 Arduino attachInterrupt has no parameter, lists of objects
// and callbacks corresponding to each possible list index have to be defined
static ICACHE_RAM_ATTR RN52* ObjList[RN52_MAX_INSTS];

template<int I> void ICACHE_RAM_ATTR sws_isr() {
	RN52::rxRead(ObjList[I]);
}

template <int N, int I = N - 1> class ISRTable : public ISRTable<N, I - 1> {
public:
	static const int dummy;
};

template <int N> class ISRTable<N, -1> {
public:
	static const int dummy;
	static void (*array[N])();
};

template <int N, int I>	const int ISRTable<N, I>::dummy =
	reinterpret_cast<int>(ISRTable<N, -1>::array[I] = sws_isr<I>) + 0 * ISRTable<N, I - 1>::dummy;

template <int N> void (*ISRTable<N, -1>::array[N])();

template class ISRTable<RN52_MAX_INSTS>;

static void (*(*ISRList))() = ISRTable<RN52_MAX_INSTS>::array;
#endif

RN52::RN52(
	int receivePin, int transmitPin, bool inverse_logic, int bufSize, int isrBufSize) {
	m_isrBuffer = 0;
	m_isrOverflow = false;
	m_isrLastCycle = 0;
	m_oneWire = (receivePin == transmitPin);
	m_invert = inverse_logic;
	if (isValidGPIOpin(receivePin)) {
		m_rxPin = receivePin;
		m_bufSize = bufSize;
		m_buffer = (uint8_t*)malloc(m_bufSize);
		m_isrBufSize = isrBufSize ? isrBufSize : 10 * bufSize;
		m_isrBuffer = static_cast<std::atomic<uint32_t>*>(malloc(m_isrBufSize * sizeof(uint32_t)));
	}
	if (isValidGPIOpin(transmitPin)
#ifdef ESP8266
		|| (!m_oneWire && (transmitPin == 16))) {
#else
		) {
#endif
		m_txValid = true;
		m_txPin = transmitPin;
	}
}

RN52::~RN52() {
	end();
	if (m_buffer) {
		free(m_buffer);
	}
	if (m_isrBuffer) {
		free(m_isrBuffer);
	}
}

bool RN52::isValidGPIOpin(int pin) {
#ifdef ESP8266
	return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 15);
#endif
#ifdef ESP32
	return pin == 0 || pin == 2 || (pin >= 4 && pin <= 5) || (pin >= 12 && pin <= 19) ||
		(pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin >= 32 && pin <= 35);
#endif
}

#ifndef ESP32
bool RN52::begin(int32_t baud, RN52Config config) {
	if (m_swsInstsIdx < 0)
		for (size_t i = 0; i < (sizeof ObjList / sizeof ObjList[0]); ++i)
		{
			if (!ObjList[i]) {
				m_swsInstsIdx = i;
				ObjList[m_swsInstsIdx] = this;
				break;
			}
		}
	if (m_swsInstsIdx < 0) return false;
#else
	void RN52::begin(int32_t baud, RN52Config config) {
#endif
	m_dataBits = 5 + (config % 4);
	m_bitCycles = ESP.getCpuFreqMHz() * 1000000 / baud;
	m_intTxEnabled = true;
	if (m_buffer != 0 && m_isrBuffer != 0) {
		m_rxValid = true;
		m_inPos = m_outPos = 0;
		m_isrInPos.store(0);
		m_isrOutPos.store(0);
		pinMode(m_rxPin, INPUT);
	}
	if (m_txValid && !m_oneWire) {
		pinMode(m_txPin, OUTPUT);
		digitalWrite(m_txPin, !m_invert);
	}

	if (!m_rxEnabled) { enableRx(true); }
#ifndef ESP32
	return true;
#endif
}

void RN52::end()
{
	enableRx(false);
#ifndef ESP32
	if (m_swsInstsIdx >= 0)	{
		ObjList[m_swsInstsIdx] = 0;
		m_swsInstsIdx = -1;
	}
#endif
}

int32_t RN52::baudRate() {
	return ESP.getCpuFreqMHz() * 1000000 / m_bitCycles;
}

void RN52::setTransmitEnablePin(int transmitEnablePin) {
	if (isValidGPIOpin(transmitEnablePin)) {
		m_txEnableValid = true;
		m_txEnablePin = transmitEnablePin;
		pinMode(m_txEnablePin, OUTPUT);
		digitalWrite(m_txEnablePin, LOW);
	} else {
		m_txEnableValid = false;
	}
}

void RN52::enableIntTx(bool on) {
	m_intTxEnabled = on;
}

void RN52::enableTx(bool on) {
	if (m_txValid && m_oneWire) {
		if (on) {
			enableRx(false);
			pinMode(m_txPin, OUTPUT);
			digitalWrite(m_txPin, !m_invert);
		} else {
			pinMode(m_rxPin, INPUT);
			enableRx(true);
		}
	}
}

void RN52::enableRx(bool on) {
	if (m_rxValid) {
		if (on) {
			m_rxCurBit = m_dataBits;
#ifndef ESP32
			attachInterrupt(digitalPinToInterrupt(m_rxPin), ISRList[m_swsInstsIdx], CHANGE);
#else
			attachInterruptArg(digitalPinToInterrupt(m_rxPin), reinterpret_cast<void (*)(void*)>(rxRead), this, CHANGE);
#endif
		} else {
			detachInterrupt(digitalPinToInterrupt(m_rxPin));
		}
		m_rxEnabled = on;
	}
}

int RN52::read() {
	if (!m_rxValid) { return -1; }
	if (m_inPos == m_outPos) {
		rxBits();
		if (m_inPos == m_outPos) { return -1; }
	}
	uint8_t ch = m_buffer[m_outPos];
	m_outPos = (m_outPos + 1) % m_bufSize;
	return ch;
}

int RN52::available() {
	if (!m_rxValid) { return 0; }
	rxBits();
	int avail = m_inPos - m_outPos;
	if (avail < 0) { avail += m_bufSize; }
	if (!avail) {
		optimistic_yield(2 * (m_dataBits + 2) * m_bitCycles / ESP.getCpuFreqMHz());
		rxBits();
		avail = m_inPos - m_outPos;
		if (avail < 0) { avail += m_bufSize; }
	}
	return avail;
}

void ICACHE_RAM_ATTR RN52::preciseDelay(uint32_t deadline, bool asyn) {
	// Reenable interrupts while delaying to avoid other tasks piling up
	if (asyn && !m_intTxEnabled) { interrupts(); }
	int32_t micro_s = static_cast<int32_t>(deadline - ESP.getCycleCount()) / ESP.getCpuFreqMHz();
	if (micro_s > 0) {
		if (asyn) optimistic_yield(micro_s); else delayMicroseconds(micro_s);
	}
	while (static_cast<int32_t>(deadline - ESP.getCycleCount()) > 0) { if (asyn) optimistic_yield(1); }
	if (asyn) {
		// Disable interrupts again
		if (!m_intTxEnabled) {
			noInterrupts();
		}
		m_periodDeadline = ESP.getCycleCount();
	}
}

void ICACHE_RAM_ATTR RN52::writePeriod(uint32_t dutyCycle, uint32_t offCycle, bool withStopBit) {
	if (dutyCycle) {
		digitalWrite(m_txPin, HIGH);
		m_periodDeadline += dutyCycle;
		preciseDelay(m_periodDeadline, withStopBit && !m_invert);
	}
	if (offCycle) {
		digitalWrite(m_txPin, LOW);
		m_periodDeadline += offCycle;
		preciseDelay(m_periodDeadline, withStopBit && m_invert);
	}
}

size_t ICACHE_RAM_ATTR RN52::write(uint8_t b) {
	return write(&b, 1);
}

size_t ICACHE_RAM_ATTR RN52::write(const uint8_t *buffer, size_t size) {
	if (m_rxValid) { rxBits(); }
	if (!m_txValid) { return 0; }

	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, HIGH);
	}
	// Stop bit : LOW if inverted logic, otherwise HIGH
	bool b = !m_invert;
	// Force line level on entry
	uint32_t dutyCycle = b;
	uint32_t offCycle = m_invert;
	// Disable interrupts in order to get a clean transmit timing
	if (!m_intTxEnabled) { noInterrupts(); }
	m_periodDeadline = ESP.getCycleCount();
	const uint32_t dataMask = ((1UL << m_dataBits) - 1);
	for (size_t cnt = 0; cnt < size; ++cnt, ++buffer) {
		bool withStopBit = true;
		// push LSB start-data-stop bit pattern into uint32_t
		// Stop bit : LOW if inverted logic, otherwise HIGH
		uint32_t word = (!m_invert) << m_dataBits;
		word |= (m_invert ? ~*buffer : *buffer) & dataMask;
		// Start bit : HIGH if inverted logic, otherwise LOW
		word <<= 1;
		word |= m_invert;
		for (int i = 0; i <= m_dataBits + 1; ++i) {
			bool pb = b;
			b = (word >> i) & 1;
			if (!pb && b) {
				writePeriod(dutyCycle, offCycle, withStopBit);
				withStopBit = false;
				dutyCycle = offCycle = 0;
			}
			if (b) {
				dutyCycle += m_bitCycles;
			} else {
				offCycle += m_bitCycles;
			}
		}
	}
	writePeriod(dutyCycle, offCycle, true);
	if (!m_intTxEnabled) { interrupts(); }
	if (m_txEnableValid) {
		digitalWrite(m_txEnablePin, LOW);
	}
	return size;
}

void RN52::flush() {
	m_inPos = m_outPos = 0;
	m_isrInPos.store(0);
	m_isrOutPos.store(0);
}

bool RN52::overflow() {
	bool res = m_overflow;
	m_overflow = false;
	return res;
}

int RN52::peek() {
	if (!m_rxValid || (rxBits(), m_inPos == m_outPos)) { return -1; }
	return m_buffer[m_outPos];
}

void ICACHE_RAM_ATTR RN52::rxBits() {
	int avail = m_isrInPos.load() - m_isrOutPos.load();
	if (avail < 0) { avail += m_isrBufSize; }
	if (m_isrOverflow.load()) {
		m_overflow = true;
		m_isrOverflow.store(false);
	}

	// stop bit can go undetected if leading data bits are at same level
	// and there was also no next start bit yet, so one byte may be pending.
	// low-cost check first
	if (avail == 0 && m_rxCurBit < m_dataBits && m_isrInPos.load() == m_isrOutPos.load() && m_rxCurBit >= 0) {
		uint32_t expectedCycle = m_isrLastCycle.load() + (m_dataBits + 1 - m_rxCurBit) * m_bitCycles;
		if (static_cast<int32_t>(ESP.getCycleCount() - expectedCycle) > m_bitCycles) {
			// Store inverted stop bit edge and cycle in the buffer unless we have an overflow
			// cycle's LSB is repurposed for the level bit
			int next = (m_isrInPos.load() + 1) % m_isrBufSize;
			if (next != m_isrOutPos.load()) {
				m_isrBuffer[m_isrInPos.load()].store((expectedCycle | 1) ^ !m_invert);
				m_isrInPos.store(next);
				++avail;
			} else {
				m_isrOverflow.store(true);
			}
		}
	}

	while (avail--) {
		// error introduced by edge value in LSB is negligible
		uint32_t isrCycle = m_isrBuffer[m_isrOutPos.load()].load();
		// extract inverted edge value
		bool level = (isrCycle & 1) == m_invert;
		m_isrOutPos.store((m_isrOutPos.load() + 1) % m_isrBufSize);
		int32_t cycles = static_cast<int32_t>(isrCycle - m_isrLastCycle.load() - (m_bitCycles / 2));
		if (cycles < 0) { continue; }
		m_isrLastCycle.store(isrCycle);
		do {
			// data bits
			if (m_rxCurBit >= -1 && m_rxCurBit < (m_dataBits - 1)) {
				if (cycles >= m_bitCycles) {
					// preceding masked bits
					int hiddenBits = cycles / m_bitCycles;
					if (hiddenBits >= m_dataBits - m_rxCurBit) { hiddenBits = (m_dataBits - 1) - m_rxCurBit; }
					bool lastBit = m_rxCurByte & 0x80;
					m_rxCurByte >>= hiddenBits;
					// masked bits have same level as last unmasked bit
					if (lastBit) { m_rxCurByte |= 0xff << (8 - hiddenBits); }
					m_rxCurBit += hiddenBits;
					cycles -= hiddenBits * m_bitCycles;
				}
				if (m_rxCurBit < (m_dataBits - 1)) {
					++m_rxCurBit;
					cycles -= m_bitCycles;
					m_rxCurByte >>= 1;
					if (level) { m_rxCurByte |= 0x80; }
				}
				continue;
			}
			if (m_rxCurBit == (m_dataBits - 1)) {
				++m_rxCurBit;
				cycles -= m_bitCycles;
				// Store the received value in the buffer unless we have an overflow
				int next = (m_inPos + 1) % m_bufSize;
				if (next != m_outPos) {
					m_buffer[m_inPos] = m_rxCurByte >> (8 - m_dataBits);
					// reset to 0 is important for masked bit logic
					m_rxCurByte = 0;
					m_inPos = next;
				} else {
					m_overflow = true;
				}
				continue;
			}
			if (m_rxCurBit >= m_dataBits) {
				// start bit level is low
				if (!level) {
					m_rxCurBit = -1;
				}
			}
			break;
		} while (cycles >= 0);
	}
}

void ICACHE_RAM_ATTR RN52::rxRead(RN52* self) {
	uint32_t curCycle = ESP.getCycleCount();
	bool level = digitalRead(self->m_rxPin);

	// Store inverted edge value & cycle in the buffer unless we have an overflow
	// cycle's LSB is repurposed for the level bit
	int next = (self->m_isrInPos.load() + 1) % self->m_isrBufSize;
	if (next != self->m_isrOutPos.load()) {
		self->m_isrBuffer[self->m_isrInPos.load()].store((curCycle | 1) ^ level);
		self->m_isrInPos.store(next);
	} else {
		self->m_isrOverflow.store(true);
	}
}

void RN52::onReceive(std::function<void(int available)> handler) {
	receiveHandler = handler;
}

void RN52::perform_work() {
	if (!m_rxValid) { return; }
	rxBits();
	if (receiveHandler) {
		int avail = m_inPos - m_outPos;
		if (avail < 0) { avail += m_bufSize; }
		if (avail) { receiveHandler(avail); }
	}
}
//Writes outputs high or low, if inputs enables/disables internal pullup
void RN52::GPIODigitalWrite(int pin, bool state)
{
  if (state) IOState = IOState | (1 << pin);
  else {
    short mask = 65535 ^ (1 << pin);
    IOState = IOState & mask;
  }
  short toWrite = (IOState | IOStateMask) & IOStateProtect;
  print("I&,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256) print("0");
  if (toWrite < 16) print("0");
  println(toWrite, HEX);
  delay(50);
}

//reads back the current state of the GPIO
bool RN52::GPIODigitalRead(int pin)
{
  while (available() > 0)
  {
    char c = read();
  }
  println("I&");
  delay(100);
  short valueIn = 0;
  for (int i = 0; i < 4; i++)
  {
    while (available() == 0);
    char c = read();
    if (c >= '0' && c <= '9')
    {
      valueIn *= 16;
      valueIn += (c - '0');
    }
    else if (c >= 'A' && c <= 'F')
    {
      valueIn *= 16;
      valueIn += (c - 'A') + 10;
    }
  }
  return (valueIn & (1 << pin)) >> pin;
}

void RN52::setDiscoverability(bool discoverable)
{
  print("@,");
  println(discoverable);
  delay(50);
}

void RN52::toggleEcho()
{
  println("+");
  delay(50);
}

void RN52::name(String nom, bool normalized)
{
  print("S");
  if (normalized) print("-,");
  else print("N,");
  println(nom);
  delay(50);
}

String RN52::name(void)
{
  while (available() > 0)
  {
    char c = read();
  }
  println("GN");
  String nom;
  char c;
  while (available() == 0);
 do
  {
    c = read();
    nom = nom + c;
  }
  while (c != '\r');
  int len=nom.length();
  nom.remove(len - 1);
  return nom;
}

void RN52::factoryReset()
{
  println("SF,1");
  delay(50);
}

int RN52::idlePowerDownTime(void)
{
  while (available() > 0)
  {
    char c = read();
  }
  int timer = 0;
  println("G^");
  while (available() == 0);
  delay(50);
  while (available() > 2)
  {
    char c = read();
    timer *= 10;
    timer += (c - '0');
    delay(50);
  }
  return timer;
}

void RN52::idlePowerDownTime(int timer)
{
  print("S^,");
  println(timer);
  delay(50);
}

void RN52::reboot()
{
  println("R,1");
  delay(2000);
}

void RN52::call(String number)
{
  print("A,");
  println(number);
  delay(50);
}

void RN52::endCall()
{
  println("E");
  delay(50);
}

void RN52::playPause()
{
  println("AP");
  delay(50);
}

void RN52::nextTrack()
{
  println("AT+");
  delay(50);
}

void RN52::prevTrack()
{
  println("AT-");
  delay(50);
}

//Credit to Greg Shuttleworth for assistance on this function
String RN52::getMetaData()
{
  while (available() > 0)
  {
    char c = read();
  }
  println("AD");
  while (available() == 0);
  String metaData;
  int i = 8;
  long count;
  while (i != 0)
  {
    if (available() > 0)
    {
      char c = read();
      count = millis();
      metaData += c;
      if (c == '\n') i--;
    }
    if ((millis() - count) > 500) i--;
  }
  return metaData;
}

String RN52::trackTitle()
{
  String metaData = getMetaData();
  int n = metaData.indexOf("Title=") + 6;
  if (n != -1) {
    metaData.remove(0, n);
    n = metaData.indexOf('\r');
    metaData.remove(n);
  }
  else metaData = "";
  return metaData;
}

String RN52::album()
{
  String metaData = getMetaData();
  int n = metaData.indexOf("Album=") + 6;
  if (n != -1) {
    metaData.remove(0, n);
    n = metaData.indexOf('\r');
    metaData.remove(n);
  }
  else metaData = "";
  return metaData;
}

String RN52::artist()
{
  String metaData = getMetaData();
  int n = metaData.indexOf("Artist=") + 7;
  if (n != -1) {
    metaData.remove(0, n);
    n = metaData.indexOf('\r');
    metaData.remove(n);
  }
  else metaData = "";
  return metaData;
}

String RN52::genre()
{
  String metaData = getMetaData();
  int n = metaData.indexOf("Genre=") + 6;
  if (n != -1) {
    metaData.remove(0, n);
    n = metaData.indexOf('\r');
    metaData.remove(n);
  }
  else metaData = "";
  return metaData;
}

int RN52::trackNumber()
{
  int trackNumber = 0;
  int attempts = 0;

  String metaData = "";

  while(trackNumber==0 && attempts < 5){
      metaData = getMetaData();
      int n = metaData.indexOf("TrackNumber=") + 12;
      if (n != -1) {
        metaData.remove(0, n);
        n = metaData.indexOf('\r');
        metaData.remove(n);
        trackNumber = metaData.toInt();
      }
    else trackNumber = 0;
    attempts++;
  }

  return trackNumber;
}

int RN52::trackCount()
{
  int trackCount = 0;
  int attempts = 0;

  String metaData = "";

  while(trackCount==0 && attempts < 5){
    metaData = getMetaData();
    int n = metaData.indexOf("TrackCount=") + 11;
    if (n != -1) {
      metaData.remove(0, n);
      n = metaData.indexOf('\r');
      metaData.remove(n);
      trackCount = metaData.toInt();
    }
    else trackCount = 0;
    attempts++;
  }
  return trackCount;
}

String RN52::getConnectionData()
{
  while (available() > 0)
  {
    char c = read();
  }
  println("D");
  while (available() == 0);
  String connectionData;
  int i = 13;
  long count;
  while (i != 0)
  {
    if (available() > 0)
    {
      char c = read();
      count = millis();
      connectionData += c;
      if (c == '\n') i--;
    }
    if ((millis() - count) > 500) i--;
  }
  return connectionData;
}

String RN52::connectedMAC()
{
  String connectionData="";
  int attempts=0;

  while(connectionData.length()!=12 && attempts < 5){

    connectionData = getConnectionData();

    int n = connectionData.indexOf("BTAC=") + 5;
    if (n != -1) {
      connectionData.remove(0, n);
      n = connectionData.indexOf('\r');
      connectionData.remove(n);
    }
    attempts++;
  }

  if(connectionData.length()!= 12){
    connectionData = "Invalid MAC Address";
  }
  return connectionData;
}

short RN52::getExtFeatures()
{
  while (available() > 0)
  {
    char c = read();
  }
  short valueIn = 0;
  char c;
  while (c != '\r')
  {
    while (available() == 0) {
      println("G%");
      delay(50);
    }
    c = read();
    if (c >= '0' && c <= '9')
    {
      valueIn *= 16;
      valueIn += (c - '0');
    }
    else if (c >= 'A' && c <= 'F')
    {
      valueIn *= 16;
      valueIn += (c - 'A') + 10;
    }
    else if ((c == '?') || (c == '!'))
    {
      while (available() > 0)
      {
        char d = read();
      }
    }
  }
  return valueIn;
}

/* <EXPERIMENTAL Q Command Stuff> */

short RN52::getEventReg()
{
  while (available() > 0)
    char c = read();

  short valueIn = 0;
  char c;

  while (c != '\r')
  {
    while (available() == 0) {
      println("Q");
      delay(50);
    }

    c = read();

    if (c >= '0' && c <= '9')
    {
      valueIn *= 16;
      valueIn += (c - '0');
    }
    else if (c >= 'A' && c <= 'F')
    {
      valueIn *= 16;
      valueIn += (c - 'A') + 10;
    }

    else if ((c == '?') || (c == '!'))
    {
      while (available() > 0)
      {
        char d = read();
      }
    }
  }

  /* Record the track change internally */
  if(!_trackChanged && (valueIn & (1 << 13)))
	  _trackChanged = true;

  return valueIn;
}

bool RN52::trackChanged(void)
{
	/* If the track has changed since this function was last called
	   and getEventReg has been called since */
	if(_trackChanged)
	{
		_trackChanged = false;
		return true;
	}

	bool change = (getEventReg() & (1 << 13));
	_trackChanged = false;
	return change;
}

bool RN52::isConnected(void)
{
	return (getEventReg() & 0x0F00);
}

/* </EXPERIMENTAL Q Command Stuff> */

void RN52::setExtFeatures(bool state, int bit)
{
  short toWrite;
  if (state) toWrite = getExtFeatures() | (1 << bit);
  else toWrite = getExtFeatures() & (65535 ^ (1 << bit));
  print("S%,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256)  print("0");
  if (toWrite < 16)   print("0");
  println(toWrite, HEX);
  delay(100);
}

void RN52::setExtFeatures(short settings)
{
  short toWrite = settings;
  print("S%,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256)  print("0");
  if (toWrite < 16)   print("0");
  println(toWrite, HEX);
  delay(100);
}

bool RN52::AVRCPButtons()
{
  return (getExtFeatures() & 1);
}

void RN52::AVRCPButtons(bool state)
{
  setExtFeatures(state, 0);
}

bool RN52::powerUpReconnect()
{
  return (getExtFeatures() & (1 << 1));
}

void RN52::powerUpReconnect(bool state)
{
  setExtFeatures(state, 1);
}

bool RN52::startUpDiscoverable()
{
  return (getExtFeatures() & (1 << 2));
}

void RN52::startUpDiscoverable(bool state)
{
  setExtFeatures(state, 2);
}

bool RN52::rebootOnDisconnect()
{
  return (getExtFeatures() & (1 << 4));
}

void RN52::rebootOnDisconnect(bool state)
{
  setExtFeatures(state, 4);
}

bool RN52::volumeToneMute()
{
  return (getExtFeatures() & (1 << 5));
}

void RN52::volumeToneMute(bool state)
{
  setExtFeatures(state, 5);
}

bool RN52::systemTonesDisabled()
{
  return (getExtFeatures() & (1 << 7));
}

void RN52::systemTonesDisabled(bool state)
{
  setExtFeatures(state, 7);
}

bool RN52::powerDownAfterPairingTimeout()
{
  return (getExtFeatures() & (1 << 8));
}

void RN52::powerDownAfterPairingTimeout(bool state)
{
  setExtFeatures(state, 8);
}

bool RN52::resetAfterPowerDown()
{
  return (getExtFeatures() & (1 << 9));
}

void RN52::resetAfterPowerDown(bool state)
{
  setExtFeatures(state, 9);
}

bool RN52::reconnectAfterPanic()
{
  return (getExtFeatures() & (1 << 10));
}

void RN52::reconnectAfterPanic(bool state)
{
  setExtFeatures(state, 10);
}

bool RN52::trackChangeEvent()
{
  return (getExtFeatures() & (1 << 12));
}

void RN52::trackChangeEvent(bool state)
{
  setExtFeatures(state, 12);
}

bool RN52::tonesAtFixedVolume()
{
  return (getExtFeatures() & (1 << 13));
}

void RN52::tonesAtFixedVolume(bool state)
{
  setExtFeatures(state, 13);
}

bool RN52::autoAcceptPasskey()
{
  return (getExtFeatures() & (1 << 14));
}

void RN52::autoAcceptPasskey(bool state)
{
  setExtFeatures(state, 14);
}

int RN52::volumeOnStartup(void)
{
  while (available() > 0)
  {
    char c = read();
  }
  println("GS");
  delay(100);
  int vol = 0;
  char c;
  while (c != '\r')
  {
    while (available() == 0);
    c = read();
    if (c >= '0' && c <= '9')
    {
      vol *= 16;
      vol += (c - '0');
    }
    else if (c >= 'A' && c <= 'F')
    {
      vol *= 16;
      vol += (c - 'A') + 10;
    }
  }
  return vol;
}

void RN52::volumeOnStartup(int vol)
{
  print("SS,");
  print("0");
  println(vol, HEX);
  delay(50);
}

void RN52::volumeUp(void)
{
  println("AV+");
  delay(50);
}

void RN52::volumeDown(void)
{
  println("AV-");
  delay(50);
}

short RN52::getAudioRouting()
{
  while (available() > 0)
  {
    char c = read();
  }
  println("G|");
  delay(100);
  short routing = 0;
  char c;
  while (c != '\r')
  {
    while (available() == 0);
    c = read();
    if (c >= '0' && c <= '9')
    {
      routing *= 16;
      routing += (c - '0');
    }
    else if (c >= 'A' && c <= 'F')
    {
      routing *= 16;
      routing += (c - 'A') + 10;
    }
  }
  return routing;
}

int RN52::sampleWidth()
{
  int width = (getAudioRouting() & 0x00F0) >> 4;
  return width;
}

void RN52::sampleWidth(int width)
{
  short mask = getAudioRouting() & 0xFF0F;
  short toWrite = mask | (width << 4);
  print("S|,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256) print("0");
  if (toWrite < 16) print("0");
  println(toWrite, HEX);
  delay(50);
}

int RN52::sampleRate()
{
  int rate = getAudioRouting() & 0x000F;
  return rate;
}

void RN52::sampleRate(int rate)
{
  short mask = getAudioRouting() & 0xFFF0;
  short toWrite = mask | rate;
  print("S|,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256) print("0");
  if (toWrite < 16) print("0");
  println(toWrite, HEX);
  delay(50);
}

int RN52::A2DPRoute()
{
  int route = (getAudioRouting() & 0x0F00) >> 8;
  return route;
}

void RN52::A2DPRoute(int route)
{
  short mask = getAudioRouting() & 0x00FF;
  short toWrite = mask | (route << 8);
  print("S|,");
  if (toWrite < 4096) print("0");
  if (toWrite < 256) print("0");
  if (toWrite < 16) print("0");
  println(toWrite, HEX);
  delay(50);
}
