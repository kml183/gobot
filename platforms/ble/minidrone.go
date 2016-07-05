package ble

import (
	"time"
	"bytes"
	"encoding/binary"
	"fmt"

	"github.com/hybridgroup/gobot"
)

var _ gobot.Driver = (*BLEMinidroneDriver)(nil)

type BLEMinidroneDriver struct {
	name       string
	connection gobot.Connection
	stepsfa0a uint16
	stepsfa0b uint16
	stepsfa0c uint16
	flying bool
	Pcmd Pcmd
	gobot.Eventer
}

const (
	// service IDs
	DroneCommandService = "9a66fa000800919111e4012d1540cb8e"
	DroneNotificationService = "9a66fb000800919111e4012d1540cb8e"

	// characteristic IDs
	PcmdCharacteristic = "9a66fa0a0800919111e4012d1540cb8e"
	CommandCharacteristic = "9a66fa0b0800919111e4012d1540cb8e"
	FlightStatusCharacteristic = "9a66fb0e0800919111e4012d1540cb8e"
	BatteryCharacteristic = "9a66fb0f0800919111e4012d1540cb8e"

	// Battery event
	Battery = "battery"

	// flight status event
	Status = "status"

	// flying event
	Flying = "flying"

	// landed event
	Landed = "landed"
)

type Pcmd struct {
	Flag  int
	Roll  int
	Pitch int
	Yaw   int
	Gaz   int
	Psi   float32
}

func validatePitch(val int) int {
	if val > 100 {
		return 100
	} else if val < 0 {
		return 0
	}

	return val
}

// NewBLEMinidroneDriver creates a BLEMinidroneDriver by name
func NewBLEMinidroneDriver(a *BLEAdaptor, name string) *BLEMinidroneDriver {
	n := &BLEMinidroneDriver{
		name:       name,
		connection: a,
		Pcmd: Pcmd{
			Flag:  0,
			Roll:  0,
			Pitch: 0,
			Yaw:   0,
			Gaz:   0,
			Psi:   0,
		},
		Eventer:    gobot.NewEventer(),
	}

	n.AddEvent(Battery)
	n.AddEvent(Status)
	n.AddEvent(Flying)
	n.AddEvent(Landed)

	return n
}
func (b *BLEMinidroneDriver) Connection() gobot.Connection { return b.connection }
func (b *BLEMinidroneDriver) Name() string                 { return b.name }

// adaptor returns BLE adaptor
func (b *BLEMinidroneDriver) adaptor() *BLEAdaptor {
	return b.Connection().(*BLEAdaptor)
}

// Start tells driver to get ready to do work
func (b *BLEMinidroneDriver) Start() (errs []error) {
	return
}

// Halt stops minidrone driver (void)
func (b *BLEMinidroneDriver) Halt() (errs []error) {
	b.Land()

	<-time.After(500 * time.Millisecond)
	return
}

func (b *BLEMinidroneDriver) Init() (err error) {
	b.GenerateAllStates()

	// subscribe to battery notifications
	b.adaptor().Subscribe(DroneNotificationService, BatteryCharacteristic, func(data []byte, e error) {
			gobot.Publish(b.Event(Battery), data[len(data)-1])
	})

	// subscribe to flying status notifications
	b.adaptor().Subscribe(DroneNotificationService, FlightStatusCharacteristic, func(data []byte, e error) {
			if len(data) < 7 || data[2] != 2 {
				fmt.Println(data)
				return
			}
			gobot.Publish(b.Event(Status), data[6])
			if (data[6] == 1 || data[6] == 2) && !b.flying {
				b.flying = true
				gobot.Publish(b.Event(Flying), true)
			} else if (data[6] == 0) && b.flying {
				b.flying = false
				gobot.Publish(b.Event(Landed), true)
			}
	})

	return
}

func (b *BLEMinidroneDriver) GenerateAllStates() (err error) {
	b.stepsfa0b++
	buf := []byte{0x04, byte(b.stepsfa0b), 0x00, 0x04, 0x01, 0x00, 0x32, 0x30, 0x31, 0x34, 0x2D, 0x31, 0x30, 0x2D, 0x32, 0x38, 0x00}
	err = b.adaptor().WriteCharacteristic(DroneCommandService, CommandCharacteristic, buf)
	if err != nil {
		fmt.Println("GenerateAllStates error:", err)
		return err
	}

	return
}

func (b *BLEMinidroneDriver) TakeOff() (err error) {
	b.stepsfa0b++
	buf := []byte{0x02, byte(b.stepsfa0b) & 0xff, 0x02, 0x00, 0x01, 0x00}
	err = b.adaptor().WriteCharacteristic(DroneCommandService, CommandCharacteristic, buf)
	if err != nil {
		fmt.Println("takeoff error:", err)
		return err
	}

	return
}

func (b *BLEMinidroneDriver) Land() (err error) {
	b.stepsfa0b++
	buf := []byte{0x02, byte(b.stepsfa0b), 0x02, 0x00, 0x03, 0x00}
	err = b.adaptor().WriteCharacteristic(DroneCommandService, CommandCharacteristic, buf)

	return err
}

func (b *BLEMinidroneDriver) FlatTrim() (err error) {
	b.stepsfa0b++
	buf := []byte{0x02, byte(b.stepsfa0b) & 0xff, 0x02, 0x00, 0x00, 0x00}
	err = b.adaptor().WriteCharacteristic(DroneCommandService, CommandCharacteristic, buf)

	return err
}

func (b *BLEMinidroneDriver) StartPcmd() {
	go func() {
		// wait a little bit so that there is enough time to get some ACKs
		<-time.After(500 * time.Millisecond)
		for {
			err := b.adaptor().WriteCharacteristic(DroneCommandService, PcmdCharacteristic, b.generatePcmd().Bytes())
			if err != nil {
				fmt.Println("pcmd write error:", err)
			}
			<-time.After(50 * time.Millisecond)
		}
	}()
}

func (b *BLEMinidroneDriver) Up(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Gaz = validatePitch(val)
	return nil
}

func (b *BLEMinidroneDriver) Down(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Gaz = validatePitch(val) * -1
	return nil
}

func (b *BLEMinidroneDriver) Forward(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Pitch = validatePitch(val)
	return nil
}

func (b *BLEMinidroneDriver) Backward(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Pitch = validatePitch(val) * -1
	return nil
}

func (b *BLEMinidroneDriver) Right(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Roll = validatePitch(val)
	return nil
}

func (b *BLEMinidroneDriver) Left(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Roll = validatePitch(val) * -1
	return nil
}

func (b *BLEMinidroneDriver) Clockwise(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Yaw = validatePitch(val)
	return nil
}

func (b *BLEMinidroneDriver) CounterClockwise(val int) error {
	b.Pcmd.Flag = 1
	b.Pcmd.Yaw = validatePitch(val) * -1
	return nil
}

func (b *BLEMinidroneDriver) Stop() error {
	b.Pcmd = Pcmd{
		Flag:  0,
		Roll:  0,
		Pitch: 0,
		Yaw:   0,
		Gaz:   0,
		Psi:   0,
	}

	return nil
}

func (b *BLEMinidroneDriver) generatePcmd() *bytes.Buffer {
	cmd := &bytes.Buffer{}
	tmp := &bytes.Buffer{}

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(2))
	cmd.Write(tmp.Bytes())

	b.stepsfa0a++
	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.stepsfa0a))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(2))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(0))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(2))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(0))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.Pcmd.Flag))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.Pcmd.Roll))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.Pcmd.Pitch))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.Pcmd.Yaw))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint16(b.Pcmd.Gaz))
	cmd.Write(tmp.Bytes())

	tmp = &bytes.Buffer{}
	binary.Write(tmp, binary.LittleEndian, uint32(b.Pcmd.Psi))
	cmd.Write(tmp.Bytes())

	return cmd
}
