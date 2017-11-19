package sysfs

import (
	"fmt"
	"os"
	"syscall"
	"unsafe"
)

const (
	// From  /usr/include/linux/i2c-dev.h:
	// ioctl signals
	I2C_SLAVE = 0x0703
	I2C_FUNCS = 0x0705
	I2C_SMBUS = 0x0720
// claude says: this appears in /usr/include/linux/i2c-dev.h:
// #define I2C_RDWR  0x0707  /* Combined R/W transfer (one STOP only) */
// further, that is where the struct mirrored here is defined:
/* This is the structure as used in the I2C_SMBUS ioctl call */
/*
struct i2c_smbus_ioctl_data {
  __u8 read_write;
  __u8 command;
  __u32 size;
  union i2c_smbus_data *data;
};
*/
// but there is also what looks like a rdwr struct as follows:

/* This is the structure as used in the I2C_RDWR ioctl call */
//
//struct i2c_rdwr_ioctl_data {
//  struct i2c_msg *msgs; /* pointers to i2c_msgs */
//  __u32 nmsgs;      /* number of i2c_msgs */
//};

/* looking in i2c.h it seems as though the problem may be that
 * my I2C device doesn't follow the SMBus protocol and mostly
 * this library follows the SMBus protocol.
 * it might yet be possible to cobble together a 2-byte send
 * with a one-byte read in atwo messages in an i2c_rdwr_ioctl_data call
*/
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



	// Read/write markers
	I2C_SMBUS_READ  = 1
	I2C_SMBUS_WRITE = 0


	// From  /usr/include/linux/i2c.h:
	// Adapter functionality
	I2C_FUNC_SMBUS_READ_BYTE        = 0x00020000
	I2C_FUNC_SMBUS_WRITE_BYTE       = 0x00040000
	I2C_FUNC_SMBUS_READ_BYTE_DATA   = 0x00080000
	I2C_FUNC_SMBUS_WRITE_BYTE_DATA  = 0x00100000
	I2C_FUNC_SMBUS_READ_WORD_DATA   = 0x00200000
	I2C_FUNC_SMBUS_WRITE_WORD_DATA  = 0x00400000
	I2C_FUNC_SMBUS_READ_BLOCK_DATA  = 0x01000000
	I2C_FUNC_SMBUS_WRITE_BLOCK_DATA = 0x02000000
	// Transaction types
	I2C_SMBUS_BYTE             = 1
	I2C_SMBUS_BYTE_DATA        = 2
	I2C_SMBUS_WORD_DATA        = 3
	I2C_SMBUS_PROC_CALL        = 4
	I2C_SMBUS_BLOCK_DATA       = 5
	I2C_SMBUS_I2C_BLOCK_BROKEN = 6
	I2C_SMBUS_BLOCK_PROC_CALL  = 7 /* SMBus 2.0 */
	I2C_SMBUS_I2C_BLOCK_DATA   = 8 /* SMBus 2.0 */
)

type i2c_rdwr_ioctl_data struct {
	prgi2c_msg uintptr
	nmsgs uint32
}

// CLAUDE: take a look in i2c.h for the struct i2c_msg:
// /usr/include/linux/i2c.h
/*
struct i2c_msg {
  __u16 addr; // slave address      
  __u16 flags;
#define I2C_M_TEN   0x0010  // this is a ten bit chip address 
#define I2C_M_RD    0x0001  // read data, from slave to master 
#define I2C_M_STOP    0x8000  // if I2C_FUNC_PROTOCOL_MANGLING 
#define I2C_M_NOSTART   0x4000  // if I2C_FUNC_NOSTART 
#define I2C_M_REV_DIR_ADDR  0x2000  // if I2C_FUNC_PROTOCOL_MANGLING 
#define I2C_M_IGNORE_NAK  0x1000  // if I2C_FUNC_PROTOCOL_MANGLING 
#define I2C_M_NO_RD_ACK   0x0800  // if I2C_FUNC_PROTOCOL_MANGLING 
#define I2C_M_RECV_LEN    0x0400  // length will be first received byte 
  __u16 len;    // msg length       
  __u8 *buf;    // pointer to msg data      
};
*/

type i2c_msg struct {
	addr uint16
	flags uint16
	len uint16
	buf uintptr
}
type i2cSmbusIoctlData struct {
	readWrite byte
	command   byte
	size      uint32
	data      uintptr
}

type i2cDevice struct {
	file  File
	funcs uint64 // adapter functionality mask
}

// NewI2cDevice returns an io.ReadWriteCloser with the proper ioctrl given
// an i2c bus location.
func NewI2cDevice(location string) (d *i2cDevice, err error) {
	d = &i2cDevice{}

	if d.file, err = OpenFile(location, os.O_RDWR, os.ModeExclusive); err != nil {
		return
	}
	if err = d.queryFunctionality(); err != nil {
		return
	}

	return
}

func (d *i2cDevice) queryFunctionality() (err error) {
	_, _, errno := Syscall(
		syscall.SYS_IOCTL,
		d.file.Fd(),
		I2C_FUNCS,
		uintptr(unsafe.Pointer(&d.funcs)),
	)

	if errno != 0 {
		err = fmt.Errorf("Querying functionality failed with syscall.Errno %v", errno)
	}
	return
}

func (d *i2cDevice) SetAddress(address int) (err error) {
	_, _, errno := Syscall(
		syscall.SYS_IOCTL,
		d.file.Fd(),
		I2C_SLAVE,
		uintptr(byte(address)),
	)

	if errno != 0 {
		err = fmt.Errorf("Setting address failed with syscall.Errno %v", errno)
	}

	return
}

func (d *i2cDevice) Close() (err error) {
	return d.file.Close()
}

func (d *i2cDevice) ReadByte() (val byte, err error) {
	if d.funcs&I2C_FUNC_SMBUS_READ_BYTE == 0 {
		return 0, fmt.Errorf("SMBus read byte not supported")
	}

	var data uint8
	err = d.smbusAccess(I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, uintptr(unsafe.Pointer(&data)))
	return data, err
}

func (d *i2cDevice) ReadByteData(reg uint8) (val uint8, err error) {
	if d.funcs&I2C_FUNC_SMBUS_READ_BYTE_DATA == 0 {
		return 0, fmt.Errorf("SMBus read byte data not supported")
	}

	var data uint8
	err = d.smbusAccess(I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, uintptr(unsafe.Pointer(&data)))
	return data, err
}

func (d *i2cDevice) ReadWordData(reg uint8) (val uint16, err error) {
	if d.funcs&I2C_FUNC_SMBUS_READ_WORD_DATA == 0 {
		return 0, fmt.Errorf("SMBus read word data not supported")
	}

	var data uint16
	err = d.smbusAccess(I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA, uintptr(unsafe.Pointer(&data)))
	return data, err
}

func (d *i2cDevice) WriteByte(val byte) (err error) {
	if d.funcs&I2C_FUNC_SMBUS_WRITE_BYTE == 0 {
		return fmt.Errorf("SMBus write byte not supported")
	}

	err = d.smbusAccess(I2C_SMBUS_WRITE, val, I2C_SMBUS_BYTE, uintptr(0))
	return err
}

func (d *i2cDevice) WriteByteData(reg uint8, val uint8) (err error) {
	if d.funcs&I2C_FUNC_SMBUS_WRITE_BYTE_DATA == 0 {
		return fmt.Errorf("SMBus write byte data not supported")
	}

	var data = val
	err = d.smbusAccess(I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, uintptr(unsafe.Pointer(&data)))
	return err
}

func (d *i2cDevice) Read8FromReg16AtBusAddr(addrDevice uint16, wReg uint16) (bRead uint8, err error) {
	bRead, err = d.i2cWrite16Read8(addrDevice, wReg)
	return
}

func (d *i2cDevice) WriteWordData(reg uint8, val uint16) (err error) {
	if d.funcs&I2C_FUNC_SMBUS_WRITE_WORD_DATA == 0 {
		return fmt.Errorf("SMBus write word data not supported")
	}

	var data = val
	err = d.smbusAccess(I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA, uintptr(unsafe.Pointer(&data)))
	return err
}

func (d *i2cDevice) WriteBlockData(reg uint8, data []byte) (err error) {
	if len(data) > 32 {
		return fmt.Errorf("Writing blocks larger than 32 bytes (%v) not supported", len(data))
	}

	buf := make([]byte, len(data)+1)
	copy(buf[1:], data)
	buf[0] = reg

	n, err := d.file.Write(buf)

	if err != nil {
		return err
	}

	if n != len(buf) {
		return fmt.Errorf("Write to device truncated, %v of %v written", n, len(buf))
	}

	return nil
}

// Read implements the io.ReadWriteCloser method by direct I2C read operations.
func (d *i2cDevice) Read(b []byte) (n int, err error) {
	return d.file.Read(b)
}

// Write implements the io.ReadWriteCloser method by direct I2C write operations.
func (d *i2cDevice) Write(b []byte) (n int, err error) {
	return d.file.Write(b)
}

func (d *i2cDevice) smbusAccess(readWrite byte, command byte, size uint32, data uintptr) error {
	smbus := &i2cSmbusIoctlData{
		readWrite: readWrite,
		command:   command,
		size:      size,
		data:      data,
	}

	_, _, errno := Syscall(
		syscall.SYS_IOCTL,
		d.file.Fd(),
		I2C_SMBUS,
		uintptr(unsafe.Pointer(smbus)),
	)

	if errno != 0 {
		return fmt.Errorf("Failed with syscall.Errno %v", errno)
	}

	return nil
}

	/* In the general case we want to take parameters;
	 * i2cbusAccess(rgbR, rgbW byte[])
	 * we want to make a single call with a single
	 * i2c_rdwr_ioctl_data which points to two i2c_msg
	 * the first of which is a write of rgbW and the
	 * second is a read of rgbR.
	 * 
	 * We want the i2c bus to look like this:
	 * START write-rgbW START read-rgbR STOP
	 *
	 * for now, though, we'll implement reading a byte
	 * from a register with a two-byte address.
	*/

func (d *i2cDevice) i2cWrite16Read8_defunct(addrDevice uint16, wWrite uint16) (byte, error) {
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
		/* the right pointer is the address of the first element of the array
		 * but a slice has other parts to it so we pass the address of the
		 * zeroth element
		*/
		// see https://coderwall.com/p/m_ma7q/pass-go-slices-as-c-array-parameters
		prgi2c_msg: uintptr(unsafe.Pointer(&rgi2c_msg[0])),
		nmsgs: 2,
	}

/*
type i2c_rdwr_ioctl_data struct {
	prgi2c_msg *i2c_msg
	nmsgs uint32
}

type i2c_msg struct {
	addr uint16
	flags uint16
	len uint16
	buf uintptr
}

	// flags for i2c_msg.flags
	I2C_M_TEN   = 0x0010  // this is a ten bit chip address 
	I2C_M_RD    = 0x0001  // read data, from slave to master 
	I2C_M_STOP    = 0x8000  // if I2C_FUNC_PROTOCOL_MANGLING 
	I2C_M_NOSTART   = 0x4000  // if I2C_FUNC_NOSTART 
	I2C_M_REV_DIR_ADDR  = 0x2000  // if I2C_FUNC_PROTOCOL_MANGLING 
	I2C_M_IGNORE_NAK  = 0x1000  // if I2C_FUNC_PROTOCOL_MANGLING 
	I2C_M_NO_RD_ACK   = 0x0800  // if I2C_FUNC_PROTOCOL_MANGLING 
	I2C_M_RECV_LEN    = 0x0400  // length will be first received byte 
	smbus := &i2cSmbusIoctlData{
		readWrite: readWrite,
		command:   command,
		size:      size,
		data:      data,
	}
*/

	//fmt.Printf("Reg16 performing I2C_RDWR with %d message parts\n", writeRead.nmsgs)
	_, _, errno := Syscall(
		syscall.SYS_IOCTL,
		d.file.Fd(),
		I2C_RDWR,
		uintptr(unsafe.Pointer(writeRead)),
	)

	if errno != 0 {
		return byteRead, fmt.Errorf("I2C_RDWR Failed with syscall.Errno %v\nWe were trying to write to register %#x on device at address %#x", errno, wWrite, addrDevice)
	}

	return byteRead, nil
}

func (d *i2cDevice) i2cWrite16Read8(addrDevice uint16, wWrite uint16) (bRead byte, err error) {
	buf := make([]byte, 2)
	buf[0] = byte(wWrite >> 8) // MSB
	buf[1] = byte(wWrite & 0xff) // LSB

	n, err := d.file.Write(buf)

	//fmt.Printf("writing 2 bytes: %#x %#x\n", buf[0], buf[1])
	if err != nil {
		return 0, err
	}

	if n != len(buf) {
		return 0, fmt.Errorf("Write to device truncated, %v of %v written", n, len(buf))
}

	bRead, err = d.ReadByte()
	return bRead, err
}

func (d *i2cDevice) Write8ToReg16AtBusAddr(addrDevice uint16, wWrite uint16, bWrite byte) error {
	buf := make([]byte, 3)
	buf[0] = byte(wWrite >> 8) // MSB
	buf[1] = byte(wWrite & 0xff) // LSB
	buf[2] = bWrite

	n, err := d.file.Write(buf)

	//fmt.Printf("writing 3 bytes: %#x %#x %#x\n", buf[0], buf[1], buf[2])
	if err != nil {
		return err
	}

	if n != len(buf) {
		return fmt.Errorf("Write to device truncated, %v of %v written", n, len(buf))
	}

	return nil
}
