package i2c

/* go library for Adafruit VL6180 ToF Sensor breakout board
 * derived from Adafruit's C++ library by Limor Fried (below)
 * ADAFRUIT NOT RESPONSIBLE FOR MY ERRORS
 * claude@bronzenose.com
 * BSD license
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

const uint8 vl6180xDefaultI2cAddr = 0x29

const uint8 vl6180xRegIdentificationModelId = 0x000
const uint8 vl6180xRegSystemInterruptConfig = 0x014
const uint8 vl6180xRegSystemInterruptClear = 0x015
const uint8 vl6180xRegSystemFreshOutOfReset = 0x016
const uint8 vl6180xRegSysrangeStart = 0x018
const uint8 vl6180xRegSysalsStart = 0x038
const uint8 vl6180xRegSysalsAnalogueGain = 0x03F
const uint8 vl6180xRegSysalsIntegrationPeriodHi = 0x040
const uint8 vl6180xRegSysalsIntegrationPeriodLo = 0x041
const uint8 vl6180xRegResultAlsVal = 0x050
const uint8 vl6180xRegResultRangeVal = 0x062
const uint8 vl6180xRegResultRangeStatus = 0x04d
const uint8 vl6180xRegResultInterruptStatusGpio = 0x04f

const uint8 vl6180xAlsGain1 = 0x06
const uint8 vl6180xAlsGain125 = 0x05
const uint8 vl6180xAlsGain167 = 0x04
const uint8 vl6180xAlsGain25 = 0x03
const uint8 vl6180xAlsGain5 = 0x02
const uint8 vl6180xAlsGain10 = 0x01
const uint8 vl6180xAlsGain20 = 0x00
const uint8 vl6180xAlsGain40 = 0x07

const uint8 vl6180xErrorNone = 0
const uint8 vl6180xErrorSyserr1 = 1
const uint8 vl6180xErrorSyserr5 = 5
const uint8 vl6180xErrorEcefail = 6
const uint8 vl6180xErrorNoconverge = 7
const uint8 vl6180xErrorRangeignore = 8
const uint8 vl6180xErrorSnr = 11
const uint8 vl6180xErrorRawuflow = 12
const uint8 vl6180xErrorRawoflow = 13
const uint8 vl6180xErrorRangeuflow = 14
const uint8 vl6180xErrorRangeoflow = 15



type Adafruit_VL6180X struct {
name       string
connector  Connector
connection Connection
Config
gobot.Commander
}


// NewBlinkMDriver creates a new BlinkMDriver.
//
// Params:
//    conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//    i2c.WithBus(int): bus to use with this driver
//    i2c.WithAddress(int): address to use with this driver
//
func NewBlinkMDriver(a Connector, options ...func(Config)) *BlinkMDriver {
  b := &BlinkMDriver{
    name:      gobot.DefaultName("BlinkM"),
    Commander: gobot.NewCommander(),
    connector: a,
    Config:    NewConfig(),
  }

  for _, option := range options {
    option(b)
  }
/* example
 b.AddCommand("Rgb", func(params map[string]interface{}) interface{} {
    red := byte(params["red"].(float64))
    green := byte(params["green"].(float64))
    blue := byte(params["blue"].(float64))
    return b.Rgb(red, green, blue)
  })
*/

	return b
}

// Name returns the Name for the Driver
func (b *BlinkMDriver) Name() string { return b.name }

// SetName sets the Name for the Driver
func (b *BlinkMDriver) SetName(n string) { b.name = n }

// Connection returns the connection for the Driver
func (b *BlinkMDriver) Connection() gobot.Connection { return b.connection.(gobot.Connection) }

// Start starts the Driver up, and writes start command
func (b *BlinkMDriver) Start() (err error) {
  bus := b.GetBusOrDefault(b.connector.GetDefaultBus())
  address := b.GetAddressOrDefault(blinkmAddress)

  b.connection, err = b.connector.GetConnection(address, bus)
  if err != nil {
    return
  }

  if _, err := b.connection.Write([]byte("o")); err != nil {
    return err
  }
  return
}

// Halt returns true if device is halted successfully
func (b *BlinkMDriver) Halt() (err error) { return }


