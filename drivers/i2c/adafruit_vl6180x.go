package i2c
import (
	"fmt"
	"errors"
	//"syscall"
	//"unsafe"
	"container/ring"
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
/*
const (
  I2C_RDWR = 0x0707


  // flags for i2c_msg.flags
  I2C_M_TEN   = 0x0010  // this is a ten bit chip address 
  I2C_M_RD    = 0x0001  // read data, from slave to master 
  I2C_M_STOP    = 0x8000  // if I2C_FUNC_PROTOCOL_MANGLING 
  I2C_M_NOSTART   = 0x4000  // if I2C_FUNC_NOSTART 
  I2C_M_REV_DIR_ADDR  = 0x2000  // if I2C_FUNC_PROTOCOL_MANGLING 
  I2C_M_IGNORE_NAK  = 0x1000  // if I2C_FUNC_PROTOCOL_MANGLING 
  I2C_M_NO_RD_ACK   = 0x0800  // if I2C_FUNC_PROTOCOL_MANGLING 
  I2C_M_RECV_LEN    = 0x0400  // length will be first received byte 
 )
type i2c_rdwr_ioctl_data struct {
  prgi2c_msg *[]i2c_msg
  nmsgs uint32
}

type i2c_msg struct {
  addr uint16
  flags uint16
  len uint16
  buf uintptr
}
*/


const vl6180xDefaultI2cAddr uint8 = 0x29

const vl6180xRegIdentificationModelId uint16 = 0x00
const vl6180xRegSystemInterruptConfig uint16 = 0x14
const vl6180xRegSystemInterruptClear uint16 = 0x15
const vl6180xRegSystemFreshOutOfReset uint16 = 0x16
const vl6180xRegSysrangeStart uint16 = 0x18
const vl6180xRegSysalsStart uint16 = 0x38
const vl6180xRegSysalsAnalogueGain uint16 = 0x3F
const vl6180xRegSysalsIntegrationPeriodHi uint16 = 0x40
const vl6180xRegSysalsIntegrationPeriodLo uint16 = 0x41
const vl6180xRegResultAlsVal uint16 = 0x50
const vl6180xRegResultRangeVal uint16 = 0x62
const vl6180xRegResultRangeStatus uint16 = 0x4d
const vl6180xRegResultInterruptStatusGpio uint16 = 0x4f

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
address    uint16
connector  Connector
connection Connection
pringDebugStr *ring.Ring
Config
gobot.Commander
}


// NewVL6180xDriver creates a new VL6180xDriver.
//
// Params:
//    conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//    i2c.WithBus(int): bus to use with this driver
//    i2c.WithAddress(int): address to use with this driver
//
func NewVL6180xDriver(a Connector, options ...func(Config)) *VL6180xDriver {
  vl6180x := &VL6180xDriver{
    name:      gobot.DefaultName("Vl6180x"),
    Commander: gobot.NewCommander(),
    connector: a,
    Config:    NewConfig(),
		pringDebugStr: ring.New(10),
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
func (vl6180x *VL6180xDriver) Name() string { return vl6180x.name }

// SetName sets the Name for the Driver
func (vl6180x *VL6180xDriver) SetName(n string) { vl6180x.name = n }

// Connection returns the connection for the Driver
func (vl6180x *VL6180xDriver) Connection() gobot.Connection { return vl6180x.connection.(gobot.Connection) }

type ErrorWriter struct {
	con Connection
	address uint16
	cw  uint
	err error
	strDuring string
}

func NewErrorWriter(con Connection, address uint16) *ErrorWriter {
	pew := new(ErrorWriter)
	pew.con = con
	pew.address = address
	if 0 == address {
		pew.err = errors.New("please don't use 0 for the device address")
	}
	return pew
}

func (pew*ErrorWriter) checkId() {
	b := pew.Read8Register16(vl6180xRegIdentificationModelId)
	if !pew.IsOk() {
		fmt.Printf("Couldn't read chip ID: %v", pew.Error())
		//vl6180x.debugStatusf("reading the chip model ID (our first contact with it) returned err %v",  pew.Error())
		return
		}
	if 0xb4 == b {
	fmt.Println("read correct chip model")
		} else {
	fmt.Printf("!!! ERROR !!! read INCORRECT chip model %#x", b)
		}
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
		//fmt.Printf("Write: %0x  err: %v\n", b, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("WriteByte(%#X)", b)
		}
	}
}

func (pew *ErrorWriter) Write8Register8(bReg, bVal uint8) {
	if nil == pew.err {
		pew.cw++
		pew.err = pew.con.WriteByteData(bReg, bVal)
		fmt.Printf("Write8Register8(reg: %#x val: %#x)  err: %v\n", bReg, bVal, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Write8Register8(reg: %#x, val: %#x)", bReg, bVal)
		}
	}
}

func (pew *ErrorWriter) Write8Register16(wReg uint16, bVal uint8) {
	if nil == pew.err {
		// fmt.Printf("Writing 8 bits (%#x) to 16-bit register %#x on bus address %#x\n", bVal, wReg, pew.address)
		pew.err = pew.con.Write8ToReg16AtBusAddr(pew.address, wReg, bVal)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Write8Register16(reg: %#X)", wReg)
		}
	}
	return
}

func (pew *ErrorWriter) WriteRegisterWord(wReg, wVal uint16) {
	if nil == pew.err {
		pew.WriteByte(uint8(wReg >> 8))
		pew.WriteByte(uint8(wReg & 0xff))
		pew.WriteByte(uint8(wVal >> 8))
		pew.WriteByte(uint8(wVal & 0xff))
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("WriteRegisterWord(reg: %#X, val: %#X)",
				wReg, wVal)
		}
	}
}

func (pew *ErrorWriter) ReadByte() (bRead uint8) {
	if nil == pew.err {
		pew.cw++
		bRead, pew.err = pew.con.ReadByte()
fmt.Printf("Read: %0x  err: %v\n", bRead, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("ReadByte()")
		}
	}
	return
}

func (pew *ErrorWriter) ReadWord() (wRead uint16) {
	if nil == pew.err {
		bMsb := pew.ReadByte()
		bLsb := pew.ReadByte()
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("ReadByte()")
		} else {
			wRead = (uint16(bMsb) << 8 ) | uint16(bLsb)
		}
	}
	return
}

func (pew *ErrorWriter) Read8Register8(bReg uint8) (bRead uint8) {
	if nil == pew.err {
		pew.cw++
		bRead, pew.err = pew.con.ReadByteData(bReg)
		fmt.Printf("Read8Register8 reg: %#x read: %#x err: %v\n", bReg, bRead, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Read8Register8()")
		}
	}
	return
}

func (pew *ErrorWriter) Read8Register16(wReg uint16) (bRead uint8) {
	if nil == pew.err {
		// fmt.Printf("Reading 8 bits from 16-bit register %#x on bus address %#x\n", wReg, pew.address)
		bRead, pew.err = pew.con.Read8FromReg16AtBusAddr(pew.address, wReg)
		// fmt.Printf("Read %#x with error %v\n", bRead, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Read8Register16(reg: %#X)", wReg)
		}
	}
	return
}

/*
func (pew *ErrorWriter) Read8Register16(wReg uint16) (bRead uint8) {
	if nil == pew.err {
		pew.WriteByte(uint8(wReg >> 8))
		pew.WriteByte(uint8(wReg & 0xff))
		bRead = pew.ReadByte()
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Read8Register16(reg: %#X)", wReg)
		}
	}
	return
}
*/

func (pew *ErrorWriter) Read16Register8(bReg uint8) (wRead uint16) {
	if nil == pew.err {
		pew.cw++
		wRead, pew.err = pew.con.ReadWordData(bReg)
		fmt.Printf("Read16Register8 reg: %#x read: %#x err: %v\n", bReg, wRead, pew.err)
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Read16Register8()")
		}
	}
	return
}

func (pew *ErrorWriter) Read16Register16(wReg uint16) (wRead uint16) {
	if nil == pew.err {
		pew.WriteByte(uint8(wReg >> 8))
		pew.WriteByte(uint8(wReg & 0xff))
		wRead = pew.ReadWord()
		if nil != pew.err {
			pew.strDuring = fmt.Sprintf("Read16Register16(reg: %#X)", wReg)
		}
	}
	return
}

func (pew *ErrorWriter) PollRegister16(wReg uint16, fnSuccess func(uint8)(bool)) {
	for {
		bRead := pew.Read8Register16(wReg)
		//fmt.Printf("Polling register %#x, rec'd %#x, error %v\n", wReg, bRead, pew.err) 
		if !pew.IsOk() || fnSuccess(bRead) {
			return
		}
	}
}

func (pvl6180x *VL6180xDriver) debugStatusf(strF string, args ...interface{}) {
	str := fmt.Sprintf(strF, args...)
	pvl6180x.pringDebugStr.Value = str
	pvl6180x.pringDebugStr = pvl6180x.pringDebugStr.Next()
}

func (pvl6180x *VL6180xDriver) debugStatus(str string) {
	pvl6180x.pringDebugStr.Value = str
	pvl6180x.pringDebugStr = pvl6180x.pringDebugStr.Next()
}

func (pvl6180x *VL6180xDriver) DebugStatus() string {
	var str string
	pvl6180x.pringDebugStr.Do(func(i interface{}) {
		if nil != i {
			str += i.(string)
			str += "\n" }
		})
	return str
}

// Start starts the Driver up, and writes start command
func (vl6180x *VL6180xDriver) Start() (err error) {
  bus := vl6180x.GetBusOrDefault(vl6180x.connector.GetDefaultBus())
  vl6180x.address = uint16(vl6180x.GetAddressOrDefault(int(vl6180xDefaultI2cAddr)))

  vl6180x.connection, err = vl6180x.connector.GetConnection(int(vl6180x.address), bus)
	vl6180x.debugStatusf("connecting to address %#x on bus %#x returned err %v", int(vl6180x.address), bus, err)
  if err != nil {
    return
  }

	pew := NewErrorWriter(vl6180x.connection, vl6180x.address)
	b := pew.Read8Register16(vl6180xRegIdentificationModelId)
	if !pew.IsOk() {
		err = pew.Error()
		vl6180x.debugStatusf("reading the chip model ID (our first contact with it) returned err %v",  err)
		return
		}
	vl6180x.debugStatusf("read chip model %#x", b)
  if b != 0xB4 {
		//vl6180x.debugStatus("MODEL DIDN'T MATCH BUT WE'RE GOING TO PROCEED ANYWAY")
    return fmt.Errorf("Model ID mismatch. Expected 0xB4, found: %#X", b)
  } else {
		vl6180x.debugStatus("We read the correct chip model")
	}
	pew.checkId()
  loadSettings(pew);
	if !pew.IsOk() {
		vl6180x.debugStatus("error loading status")
	} else {
		vl6180x.debugStatus("loaded status successfully")
	}
	pew.checkId()
	pew.Write8Register16(vl6180xRegSystemFreshOutOfReset, 0x00)
	pew.checkId()
	return pew.Error()
}

// Halt returns true if device is halted successfully
func (vl6180x *VL6180xDriver) Halt() (err error) { return }


/**************************************************************************/
/*! 
    @brief  Load the settings for ranging
*/
/**************************************************************************/

func  loadSettings(pew*ErrorWriter) {
    // load settings!

    // private settings from page 24 of app note
    pew.Write8Register16(0x0207, 0x01)
    pew.Write8Register16(0x0208, 0x01)
    pew.Write8Register16(0x0096, 0x00)
    pew.Write8Register16(0x0097, 0xfd)
    pew.Write8Register16(0x00e3, 0x00)
    pew.Write8Register16(0x00e4, 0x04)
    pew.Write8Register16(0x00e5, 0x02)
    pew.Write8Register16(0x00e6, 0x01)
    pew.Write8Register16(0x00e7, 0x03)
    pew.Write8Register16(0x00f5, 0x02)
    pew.Write8Register16(0x00d9, 0x05)
    pew.Write8Register16(0x00db, 0xce)
    pew.Write8Register16(0x00dc, 0x03)
    pew.Write8Register16(0x00dd, 0xf8)
    pew.Write8Register16(0x009f, 0x00)
    pew.Write8Register16(0x00a3, 0x3c)
    pew.Write8Register16(0x00b7, 0x00)
    pew.Write8Register16(0x00bb, 0x3c)
    pew.Write8Register16(0x00b2, 0x09)
    pew.Write8Register16(0x00ca, 0x09)
    pew.Write8Register16(0x0198, 0x01)
    pew.Write8Register16(0x01b0, 0x17)
    pew.Write8Register16(0x01ad, 0x00)
    pew.Write8Register16(0x00ff, 0x05)
    pew.Write8Register16(0x0100, 0x05)
    pew.Write8Register16(0x0199, 0x05)
    pew.Write8Register16(0x01a6, 0x1b)
    pew.Write8Register16(0x01ac, 0x3e)
    pew.Write8Register16(0x01a7, 0x1f)
    pew.Write8Register16(0x0030, 0x00)

    // Recommended : Public registers - See data sheet for more detail
    pew.Write8Register16(0x0011, 0x10)       // Enables polling for 'New Sample ready'
                                // when measurement completes
    pew.Write8Register16(0x010a, 0x30)       // Set the averaging sample period
                                // (compromise between lower noise and
                                // increased execution time)
    pew.Write8Register16(0x003f, 0x46)       // Sets the light and dark gain (upper
                                // nibble). Dark gain should not be
                                // changed.
    pew.Write8Register16(0x0031, 0xFF)       // sets the # of range measurements after
                                // which auto calibration of system is
                                // performed
    pew.Write8Register16(0x0040, 0x63)       // Set ALS integration time to 100ms
    pew.Write8Register16(0x002e, 0x01)       // perform a single temperature calibration
                                // of the ranging sensor

    // Optional: Public registers - See data sheet for more detail
    pew.Write8Register16(0x001b, 0x09)       // Set default ranging inter-measurement
                                // period to 100ms
    pew.Write8Register16(0x003e, 0x31)       // Set default ALS inter-measurement period
                                // to 500ms
    pew.Write8Register16(0x0014, 0x24)       // Configures interrupt on 'New Sample
                                // Ready threshold event'
}

/**************************************************************************/
/*! 
    @brief  Single shot ranging
*/
/**************************************************************************/

func (pvl6180x *VL6180xDriver) ReadRange() (bRange, bStatus uint8, err error) {
	pew := NewErrorWriter(pvl6180x.connection, pvl6180x.address)
	//pew.checkId()
	//fmt.Println("polling for device ready")
  // wait for device to be ready for range measurement
	pew.PollRegister16(vl6180xRegResultRangeStatus, func(bRead uint8)(bool){
		return 0 != (bRead & 0x01)
		})
	//pew.checkId()
	//fmt.Println("Starting measurement")
  // Start a range measurement
  pew.Write8Register16(vl6180xRegSysrangeStart, 0x01)

	//pew.checkId()
  // Poll until bit 2 is set
	//fmt.Println("polling for interrupt status")
	pew.PollRegister16(vl6180xRegResultInterruptStatusGpio,
		func(bRead uint8)(bool){ return 0x04 == (bRead & 0x04) } )

  // read range in mm
	//fmt.Println("reading range")
  bRange = pew.Read8Register16(vl6180xRegResultRangeVal)

  // clear interrupt
	//fmt.Println("clearing interrupt")
  pew.Write8Register16(vl6180xRegSystemInterruptClear, 0x07)

	// read error message
	bStatus = pew.ReadRangeStatus()

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

func (pew *ErrorWriter) ReadRangeStatus() uint8 {
  return pew.Read8Register16(vl6180xRegResultRangeStatus) >> 4
}


/**************************************************************************/
/*! 
    @brief  Single shot ranging
*/
/**************************************************************************/

func (pvl6180x *VL6180xDriver) ReadLux(gain uint8) float32 {
	pew := NewErrorWriter(pvl6180x.connection, pvl6180x.address)

  var bMask uint8
  bMask = pew.Read8Register16(vl6180xRegSystemInterruptConfig)
  bMask &= ^uint8(0x38)
  bMask |= (0x4 << 3) // IRQ on ALS ready
  pew.Write8Register16(vl6180xRegSystemInterruptConfig, bMask)

  // 100 ms integration period
  pew.Write8Register16(vl6180xRegSysalsIntegrationPeriodHi, 0)
  pew.Write8Register16(vl6180xRegSysalsIntegrationPeriodLo, 100)

  // analog gain
  if gain > vl6180xAlsGain40 {
    gain = vl6180xAlsGain40
  }
  pew.Write8Register16(vl6180xRegSysalsAnalogueGain, 0x40 | gain)

  // start ALS
  pew.Write8Register16(vl6180xRegSysalsStart, 0x1)

  // Poll until "New Sample Ready threshold event" is set
	pew.PollRegister16(vl6180xRegResultInterruptStatusGpio,
		func (bRead uint8)(bool) {return 4 == ((bRead >> 3) & 0x7)})

  // read lux!
  lux := float32(pew.Read16Register16(vl6180xRegResultAlsVal))

  // clear interrupt
  pew.Write8Register16(vl6180xRegSystemInterruptClear, 0x07)

  lux *= 0.32 // calibrated count/lux
  switch(gain) {
  case vl6180xAlsGain1:
    break
  case vl6180xAlsGain125:
    lux /= 1.25
    break
  case vl6180xAlsGain167:
    lux /= 1.76
    break
  case vl6180xAlsGain25:
    lux /= 2.5
    break
  case vl6180xAlsGain5:
    lux /= 5
    break
  case vl6180xAlsGain10:
    lux /= 10
    break
  case vl6180xAlsGain20:
    lux /= 20
    break
  case vl6180xAlsGain40:
    lux /= 20
    break
  }
  //lux *= 100
  //lux /= 100 // integration time in ms


  return lux
}

/* these sort of belong in sysfs i2cDevice but that's not
 * exported - we'd need to change Connection or something
 * for now we only want to know if the method works.
 */
/*
func i2cWrite16Read8(addrDevice uint16, wWrite uint16) (byte, error) {
  var byteRead byte
  var flagsTenBit uint16 = 0
  if 0 < (addrDevice & 0xff00) {
    flagsTenBit = I2C_M_TEN
  }
  rgi2c_msg := []i2c_msg {
    i2c_msg {
      addr: addrDevice,
      flags: 0 | flagsTenBit, // write
      len:   2,
      buf: uintptr(unsafe.Pointer(&wWrite)),
    },
    i2c_msg {
      addr: addrDevice,
      flags: I2C_M_RD | flagsTenBit,
      len: 1,
      buf: uintptr(unsafe.Pointer(&byteRead)),
    },
  }
 writeRead := &i2c_rdwr_ioctl_data {
    prgi2c_msg: &rgi2c_msg,
    nmsgs: 2,
  }

  _, _, errno := Syscall(
    syscall.SYS_IOCTL,
    d.file.Fd(),
    I2C_RDWR,
    uintptr(unsafe.Pointer(writeRead)),
  )

  if errno != 0 {
    return byteRead, fmt.Errorf("I2C_RDWR Failed with syscall.Errno %v", errno)
  }

  return byteRead, nil
}
*/


