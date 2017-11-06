package i2c
import (
	"fmt"
	"gobot.io/x/gobot"
)

/* go library for Adafruit VL6180 ToF Sensor breakout board
 * derived from Adafruit's C++ library by Limor Fried (below)
 * https://github.com/adafruit/Adafruit_VL6180X/blob/master/Adafruit_VL6180X.h
 * https://github.com/adafruit/Adafruit_VL6180X/blob/master/Adafruit_VL6180X.cpp
 *
 * ADAFRUIT NOT RESPONSIBLE FOR MY ERRORS
 * claude@bronzenose.com
 * BSD license
 *
 * As Adafruit puts it:
 *	Adafruit invests time and resources providing this open source code, 
 *	please support Adafruit and open-source hardware by purchasing 
 *	products from Adafruit!
*/
/*! 
    based on file     Adafruit_VL6180X.h / .cpp
    by                Limor Fried (Adafruit Industries)
	  with license      BSD (see license.txt)
	
	This is a library for the Adafruit VL6180 ToF Sensor breakout board
	----> http://www.adafruit.com/products/3316
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!
*/
/**************************************************************************/

const vl6180xDefaultI2cAddr uint8 = 0x29

const vl6180xRegIdentificationModelId uint8 = 0x000
const vl6180xRegSystemInterruptConfig uint8 = 0x014
const vl6180xRegSystemInterruptClear uint8 = 0x015
const vl6180xRegSystemFreshOutOfReset uint8 = 0x016
const vl6180xRegSysrangeStart uint8 = 0x018
const vl6180xRegSysalsStart uint8 = 0x038
const vl6180xRegSysalsAnalogueGain uint8 = 0x03F
const vl6180xRegSysalsIntegrationPeriodHi uint8 = 0x040
const vl6180xRegSysalsIntegrationPeriodLo uint8 = 0x041
const vl6180xRegResultAlsVal uint8 = 0x050
const vl6180xRegResultRangeVal uint8 = 0x062
const vl6180xRegResultRangeStatus uint8 = 0x04d
const vl6180xRegResultInterruptStatusGpio uint8 = 0x04f

const vl6180xAlsGain1 uint8 = 0x06
const vl6180xAlsGain125 uint8 = 0x05
const vl6180xAlsGain167 uint8 = 0x04
const vl6180xAlsGain25 uint8 = 0x03
const vl6180xAlsGain5 uint8 = 0x02
const vl6180xAlsGain10 uint8 = 0x01
const vl6180xAlsGain20 uint8 = 0x00
const vl6180xAlsGain40 uint8 = 0x07

const vl6180xErrorNone uint8 = 0
const vl6180xErrorSyserr1 uint8 = 1
const vl6180xErrorSyserr5 uint8 = 5
const vl6180xErrorEcefail uint8 = 6
const vl6180xErrorNoconverge uint8 = 7
const vl6180xErrorRangeignore uint8 = 8
const vl6180xErrorSnr uint8 = 11
const vl6180xErrorRawuflow uint8 = 12
const vl6180xErrorRawoflow uint8 = 13
const vl6180xErrorRangeuflow uint8 = 14
const vl6180xErrorRangeoflow uint8 = 15



type VL6180xDriver struct {
name       string
connector  Connector
connection Connection
Config
gobot.Commander
}


// NewVl6180xDriver creates a new Vl6180xDriver.
//
// Params:
//    conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//    i2c.WithBus(int): bus to use with this driver
//    i2c.WithAddress(int): address to use with this driver
//
func NewVl6180xDriver(a Connector, options ...func(Config)) *Vl6180xDriver {
  vl6180x := &Vl6180xDriver{
    name:      gobot.DefaultName("Vl6180x"),
    Commander: gobot.NewCommander(),
    connector: a,
    Config:    NewConfig(),
  }

  for _, option := range options {
    option(vl6180x)
  }
/* example
 vl6180x.AddCommand("Rgb", func(params map[string]interface{}) interface{} {
    red := byte(params["red"].(float64))
    green := byte(params["green"].(float64))
    blue := byte(params["blue"].(float64))
    return vl6180x.Rgb(red, green, blue)
  })
*/

	return vl6180x
}

// Name returns the Name for the Driver
func (vl6180x *Vl6180xDriver) Name() string { return vl6180x.name }

// SetName sets the Name for the Driver
func (vl6180x *Vl6180xDriver) SetName(n string) { vl6180x.name = n }

// Connection returns the connection for the Driver
func (vl6180x *Vl6180xDriver) Connection() gobot.Connection { return vl6180x.connection.(gobot.Connection) }

type ErrorWriter struct {
	con Gobot.Connection
	cw  uint
	err error
	strDuring string
}

func NewErrorWriter(con Gobot.Connection) *ErrorWriter {
	pew := new(ErrorWriter)
	pew.con = con
	return pew
}

func (pew *ErrorWriter) ErrUnderlying() error {
	return pew.err
}

func (pew *ErrorWriter) IsOk() bool {
	return nil == pew.err
}

func (pew *ErrorWriter) Error() error {
	if nil == pew.err {
		return nil
	}
	return fmt.Errorf("An '%s' error occurred during the %d write operation which was: %s",
		pew.err, pew.cw, pew.strDuring)
}

func (pew *ErrorWriter) WriteByte(b uint8) {
	if nil == pew.err {
		pew.cw++
		pew.err = pew.con.WriteByte(b)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("WriteByte(%#X)", b)
		}
	}
}

func (pew *ErrorWriter) WriteRegisterByte(bReg, bVal uint8) {
	if nil == pew.err {
		pew.cw++
		pew.err = pew.con.WriteByteData(bReg, bVal)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("WriteRegisterByte(reg: %#X, val: %#X)",
				bReg, bVal)
		}
	}
}

func (pew *ErrorWriter) ReadByte() (bRead uint8) {
	if nil == pew.err {
		pew.cw++
		bRead, pew.err = pew.con.ReadByte()
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("ReadByte()")
		}
	}
	return
}

func (pew *ErrorWriter) ReadRegisterByte(bReg uint8) (bRead uint8) {
	if nil == pew.err {
		pew.cw++
		bRead, pew.err = pew.con.ReadByteData(bReg)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("ReadRegisterByte(reg: %#X)", bReg)
		}
	}
	return
}

func (pew *ErrorWriter) PollRegister(bReg uint8, fnTest func(uint8)(bool)) {
	for {
		bRead := pew.ReadRegisterByte(bReg)
		if !pew.IsOk() || fn(bRead) {
			return
		}
	}
}


// Start starts the Driver up, and writes start command
func (vl6180x *Vl6180xDriver) Start() (err error) {
  bus := vl6180x.GetBusOrDefault(vl6180x.connector.GetDefaultBus())
  address := vl6180x.GetAddressOrDefault(blinkmAddress)

  vl6180x.connection, err = vl6180x.connector.GetConnection(address, bus)
  if err != nil {
    return
  }

	pew := NewErrorWriter(vl6180x.connection)
	b := pew.ReadRegisterByte(vl6180xRegIdentificationModelId)
	if !pew.IsOk() {
		return pew.Error()
	}
  if b != 0xB4 {
    return fmt.Errorf("Model ID mismatch. Expected 0xB4, found: %#X", b)
  }

  loadSettings(pew);
  pew.WriteRegisterByte(vl6180xRegSystemFreshOutOfReset, 0x00)
	return pew.Error()
}

// Halt returns true if device is halted successfully
func (vl6180x *Vl6180xDriver) Halt() (err error) { return }


/**************************************************************************/
/*! 
    @brief  Load the settings for ranging
*/
/**************************************************************************/

func  loadSettings(pew*ErrorWriter) {
    // load settings!

    // private settings from page 24 of app note
    pew.WriteRegisterByte(0x0207, 0x01)
    pew.WriteRegisterByte(0x0208, 0x01)
    pew.WriteRegisterByte(0x0096, 0x00)
    pew.WriteRegisterByte(0x0097, 0xfd)
    pew.WriteRegisterByte(0x00e3, 0x00)
    pew.WriteRegisterByte(0x00e4, 0x04)
    pew.WriteRegisterByte(0x00e5, 0x02)
    pew.WriteRegisterByte(0x00e6, 0x01)
    pew.WriteRegisterByte(0x00e7, 0x03)
    pew.WriteRegisterByte(0x00f5, 0x02)
    pew.WriteRegisterByte(0x00d9, 0x05)
    pew.WriteRegisterByte(0x00db, 0xce)
    pew.WriteRegisterByte(0x00dc, 0x03)
    pew.WriteRegisterByte(0x00dd, 0xf8)
    pew.WriteRegisterByte(0x009f, 0x00)
    pew.WriteRegisterByte(0x00a3, 0x3c)
    pew.WriteRegisterByte(0x00b7, 0x00)
    pew.WriteRegisterByte(0x00bb, 0x3c)
    pew.WriteRegisterByte(0x00b2, 0x09)
    pew.WriteRegisterByte(0x00ca, 0x09)
    pew.WriteRegisterByte(0x0198, 0x01)
    pew.WriteRegisterByte(0x01b0, 0x17)
    pew.WriteRegisterByte(0x01ad, 0x00)
    pew.WriteRegisterByte(0x00ff, 0x05)
    pew.WriteRegisterByte(0x0100, 0x05)
    pew.WriteRegisterByte(0x0199, 0x05)
    pew.WriteRegisterByte(0x01a6, 0x1b)
    pew.WriteRegisterByte(0x01ac, 0x3e)
    pew.WriteRegisterByte(0x01a7, 0x1f)
    pew.WriteRegisterByte(0x0030, 0x00)

    // Recommended : Public registers - See data sheet for more detail
    pew.WriteRegisterByte(0x0011, 0x10)       // Enables polling for 'New Sample ready'
                                // when measurement completes
    pew.WriteRegisterByte(0x010a, 0x30)       // Set the averaging sample period
                                // (compromise between lower noise and
                                // increased execution time)
    pew.WriteRegisterByte(0x003f, 0x46)       // Sets the light and dark gain (upper
                                // nibble). Dark gain should not be
                                // changed.
    pew.WriteRegisterByte(0x0031, 0xFF)       // sets the # of range measurements after
                                // which auto calibration of system is
                                // performed
    pew.WriteRegisterByte(0x0040, 0x63)       // Set ALS integration time to 100ms
    pew.WriteRegisterByte(0x002e, 0x01)       // perform a single temperature calibration
                                // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    pew.WriteRegisterByte(0x001b, 0x09)       // Set default ranging inter-measurement
                                // period to 100ms
    pew.WriteRegisterByte(0x003e, 0x31)       // Set default ALS inter-measurement period
                                // to 500ms
    pew.WriteRegisterByte(0x0014, 0x24)       // Configures interrupt on 'New Sample
                                // Ready threshold event'
}

/**************************************************************************/
/*! 
    @brief  Single shot ranging
*/
/**************************************************************************/

func (pvl6180x *VL6180xDriver) readRange() (bStatus, bRange uint8, err error) {
	pew := NewErrorWriter(vl6180x.connection)
  // wait for device to be ready for range measurement
	pew.PollRegister(vl6180xRegResultRangeStatus, func(bRead uint8)(bool){
		return 0 != (bRead & 0x01)
		})
  // Start a range measurement
  pew.WriteRegisterByte(vl6180xRegSysrangeStart, 0x01)

  // Poll until bit 2 is set
	pew.PollRegister(vl6180xRegResultInterruptStatusGpio,
		func(bRead uint8)(bool){ return 0!= (bRead & 0x04) } )

  // read range in mm
  bRange = pew.ReadRegisterByte(vl6180xRegResultRangeVal)

  // clear interrupt
  pew.WriteRegisterByte(vl6180xRegSystemInterruptClear, 0x07)

	if !pew.IsOk() {
		err = pew.Error()
	}
	return
}


/**************************************************************************/
/*! 
    @brief  Error message (retreive after ranging)
*/
/**************************************************************************/

uint8T Adafruit_VL6180X::readRangeStatus(void) {
  return (read8(vl6180xRegResultRangeStatus) >> 4);
}


/**************************************************************************/
/*! 
    @brief  Single shot ranging
*/
/**************************************************************************/

float Adafruit_VL6180X::readLux(uint8T gain) {
  uint8T reg;

  reg = read8(vl6180xRegSystemInterruptConfig);
  reg &= ~0x38;
  reg |= (0x4 << 3); // IRQ on ALS ready
  write8(vl6180xRegSystemInterruptConfig, reg);

  // 100 ms integration period
  write8(vl6180xRegSysalsIntegrationPeriodHi, 0);
  write8(vl6180xRegSysalsIntegrationPeriodLo, 100);

  // analog gain
  if (gain > vl6180xAlsGain40) {
    gain = vl6180xAlsGain40;
  }
  write8(vl6180xRegSysalsAnalogueGain, 0x40 | gain);

  // start ALS
  write8(vl6180xRegSysalsStart, 0x1);

  // Poll until "New Sample Ready threshold event" is set
  while (4 != ((read8(vl6180xRegResultInterruptStatusGpio) >> 3) & 0x7));

  // read lux!
  float lux = read16(vl6180xRegResultAlsVal);

  // clear interrupt
  write8(vl6180xRegSystemInterruptClear, 0x07);

  lux *= 0.32; // calibrated count/lux
  switch(gain) {
  case vl6180xAlsGain1:
    break;
  case vl6180xAlsGain125:
    lux /= 1.25;
    break;
  case vl6180xAlsGain167:
    lux /= 1.76;
    break;
  case vl6180xAlsGain25:
    lux /= 2.5;
    break;
  case vl6180xAlsGain5:
    lux /= 5;
    break;
  case vl6180xAlsGain10:
    lux /= 10;
    break;
  case vl6180xAlsGain20:
    lux /= 20;
    break;
  case vl6180xAlsGain40:
    lux /= 20;
    break;
  }
  lux *= 100;
  lux /= 100; // integration time in ms


  return lux;
}

/**************************************************************************/
/*! 
    @brief  I2C low level interfacing
*/
/**************************************************************************/


// Read 1 byte from the VL6180X at 'address'
uint8_t Adafruit_VL6180X::read8(uint16_t address)
{
  uint8_t data;

  Wire.beginTransmission(_i2caddr);
  Wire.write(address>>8);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, (uint8_t)1);
  uint8_t r = Wire.read();

#if defined(I2C_DEBUG)
  Serial.print("\t$"); Serial.print(address, HEX); Serial.print(": 0x"); Serial.println(r, HEX);
#endif

  return r;
}


// Read 2 byte from the VL6180X at 'address'
uint16_t Adafruit_VL6180X::read16(uint16_t address)
{
  uint16_t data;

  Wire.beginTransmission(_i2caddr);
  Wire.write(address>>8);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, (uint8_t)2);
  while(!Wire.available());
  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
  
  return data;
}

// write 1 byte
void Adafruit_VL6180X::write8(uint16_t address, uint8_t data)
{
  Wire.beginTransmission(_i2caddr);
  Wire.write(address>>8);
  Wire.write(address);
  Wire.write(data);  
  Wire.endTransmission();

#if defined(I2C_DEBUG)
  Serial.print("\t$"); Serial.print(address, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);
#endif
}


// write 2 bytes
void Adafruit_VL6180X::write16(uint16_t address, uint16_t data)
{
  Wire.beginTransmission(_i2caddr);
  Wire.write(address>>8);
  Wire.write(address);
  Wire.write(data>>8);
  Wire.write(data);
  Wire.endTransmission();
}


