package i2c

import (
	"errors"
	"fmt"

	"gobot.io/x/gobot"
)

const (

	// Registers
	bme680RegisterResistanceHeatRange = 0x02
	bme680RegisterRangeSoftwareError  = 0x04
	bme680RegisterTempMSB             = 0x22
	bme680RegisterGasResistanceMSB    = 0x2A
	bme680RegisterCtrlGasOne          = 0x71
	bme680RegisterCtrlHumidity        = 0x72
	bme680RegisterCtrlMeasurement     = 0x74
	bme680RegisterConfig              = 0x75
	bme680RegisterCoeffAddr1          = 0x89
	bme680RegisterSoftReset           = 0xE0
	bme680RegisterCoeffAddr2          = 0xE1
	bme680RegisterChipID              = 0xD0

	// Calibration register mappings
	bme680T2LSBCalibrationReg  = 1
	bme680T2MSBCalibrationReg  = 2
	bme680T3CalibrationReg     = 3
	bme680P1LSBCalibrationReg  = 5
	bme680P1MSBCalibrationReg  = 6
	bme680P2LSBCalibrationReg  = 7
	bme680P2MSBCalibrationReg  = 8
	bme680P3CalibrationReg     = 9
	bme680P4LSBCalibrationReg  = 11
	bme680P4MSBCalibrationReg  = 12
	bme680P5LSBCalibrationReg  = 13
	bme680P5MSBCalibrationReg  = 14
	bme680P7CalibrationReg     = 15
	bme680P6CalibrationReg     = 16
	bme680P8LSBCalibrationReg  = 19
	bme680P8MSBCalibrationReg  = 20
	bme680P9LSBCalibrationReg  = 21
	bme680P9MSBCalibrationReg  = 22
	bme680P10CalibrationReg    = 23
	bme680H2MSBCalibrationReg  = 25
	bme680H2LSBCalibrationReg  = 26
	bme680H1LSBCalibrationReg  = 26
	bme680H1MSBCalibrationReg  = 27
	bme680H3CalibrationReg     = 28
	bme680H4CalibrationReg     = 29
	bme680H5CalibrationReg     = 30
	bme680H6CalibrationReg     = 31
	bme680H7CalibrationReg     = 32
	bme680T1LSBCalibrationReg  = 33
	bme680T1MSBCalibrationReg  = 34
	bme680GH2LSBCalibrationReg = 35
	bme680GH2MSBCalibrationReg = 36
	bme680GH1CalibrationReg    = 37
	bme680GH3CalibrationReg    = 38

	bme680SoftResetData = 0xB6
	bme680ChipID        = 0x61
	bme680Address       = 0x77
)

var (
	bme680GasResistanceLookupTable1 = []uint32{2147483647, 2147483647, 2147483647, 2147483647,
		2147483647, 2126008810, 2147483647, 2130303777,
		2147483647, 2147483647, 2143188679, 2136746228,
		2147483647, 2126008810, 2147483647, 2147483647}
	/**Look up table 2 for the possible gas range values */
	bme680GasResistanceLookupTable2 = []uint32{4096000000, 2048000000, 1024000000, 512000000,
		255744255, 127110228, 64000000, 32258064, 16016016,
		8000000, 4000000, 2000000, 1000000, 500000,
		250000, 125000}
)

type BME680IIRFilterCoefficient uint8

const (
	bme680ZeroFilterCoefficient                  = 0x00
	bme680OneFilterCoefficient                   = 0x01
	bme680ThreeFilterCoefficient                 = 0x02
	bme680SevenFilterCoefficient                 = 0x03
	bme680FifteenFilterCoefficient               = 0x04
	bme680ThirtyOneFilterCoefficient             = 0x05
	bme680SixtyThreeFilterCoefficient            = 0x06
	bme680OneHundredTwentySevenFilterCoefficient = 0x07
)

type BME680OverSamplingRate uint8

const (
	bme680OneTimesOverSample     = 0x01
	bme680TwoTimesOverSample     = 0x02
	bme680FourTimesOverSample    = 0x03
	bme680EightTimesOverSample   = 0x04
	bme680SixteenTimesOverSample = 0x05
)

type BME680SensorPowerMode uint8

const (
	bme680SleepMode BME680SensorPowerMode = 0x00
	bme680ForceMode BME680SensorPowerMode = 0x01
)

type BME680CalibrationCoefficients struct {
	t1 uint16
	t2 int16
	t3 int8

	gh1 int8
	gh2 int16
	gh3 int8

	resHeatRange  uint8
	softwareError uint8
}

// BME680Driver is a driver for the BME680 temperature/humidity/gas sensor.
// It implements all of the same functions as the BMP280Driver, but also
// adds the Gas() function by reading the BME680's metal oxide gas sensor.
// For details on the BMP280Driver please see:
// 	https://godoc.org/gobot.io/x/gobot/drivers/i2c#BMP280Driver
//
type BME680Driver struct {
	hc         *bmeHumidityCalibrationCoefficients
	connector  Connector
	name       string
	connection Connection
	Config
	CalibrationCoefficients *BME680CalibrationCoefficients
}

// NewBME680Driver creates a new driver with specified i2c interface.
// Params:
//		conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//		i2c.WithBus(int):	bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewBME680Driver(c Connector, options ...func(Config)) *BME680Driver {
	b := &BME680Driver{
		hc:                      &bmeHumidityCalibrationCoefficients{},
		connector:               c,
		name:                    gobot.DefaultName("BME680"),
		Config:                  NewConfig(),
		CalibrationCoefficients: &BME680CalibrationCoefficients{},
	}

	for _, option := range options {
		option(b)
	}

	return b
}

// Name returns the name of the device.
func (d *BME680Driver) Name() string {
	return d.name
}

// SetName sets the name of the device.
func (d *BME680Driver) SetName(n string) {
	d.name = n
}

// Connection returns the connection of the device.
func (d *BME680Driver) Connection() gobot.Connection {
	return d.connector.(gobot.Connection)
}

// Halt halts the device.
func (d *BME680Driver) Halt() (err error) {
	return nil
}

// Start initializes the BME280 and loads the calibration coefficients.
func (d *BME680Driver) Start() (err error) {
	bus := d.GetBusOrDefault(d.connector.GetDefaultBus())
	address := d.GetAddressOrDefault(bme680Address)

	if d.connection, err = d.connector.GetConnection(address, bus); err != nil {
		return err
	}
	if err := d.connection.WriteByteData(bme680RegisterSoftReset, bme680SoftResetData); err != nil {
		return err
	}

	chipID, err := d.connection.ReadByteData(bme680RegisterChipID)
	if err != nil {
		return err
	}

	if chipID != bme680ChipID {
		return errors.New("yte queried chip ID does not equal the known ID for the BME680; this is most likely an issue with the wired connection to the device")
	}

	// The default oversampling values suggested in the quick start section (3.2.1)
	err = d.UpdateOverSamplingRates(bme680FifteenFilterCoefficient, bme680FourTimesOverSample, bme680TwoTimesOverSample)
	if err != nil {
		return err
	}

	err = d.UpdateIIRFilterCoefficient(bme680ThreeFilterCoefficient)
	if err != nil {
		return err
	}

	err = d.setupGasSensorConfiguration()
	if err != nil {
		return err
	}

	_, err = d.GetCalibrationConstants()
	if err != nil {
		return err
	}

	return nil
}

func (d *BME680Driver) UpdateOverSamplingRates(tRate, gRate, hRate BME680OverSamplingRate) error {
	currentCtrlMeasurementVal, err := d.connection.ReadByteData(bme680RegisterCtrlMeasurement)
	if err != nil {
		return err
	}

	// Set osrs_t<2:0> and osrs_pres_heat_range<2:0> to the temperature and gas rates respectively
	// Preserve the value of moderes_heat_range<1:0>
	newCtrlMeasurementVal := (currentCtrlMeasurementVal & 0x03) | (uint8(tRate) << 5) | (uint8(gRate) << 2)

	err = d.connection.WriteByteData(bme680RegisterCtrlMeasurement, newCtrlMeasurementVal)
	if err != nil {
		return err
	}

	// Set osrs_h<2:0>
	// Unset spi_3w_int_en as this should always be 0 for the device being used with i2c
	return d.connection.WriteByteData(bme680RegisterCtrlHumidity, uint8(hRate))
}

func (d *BME680Driver) UpdateIIRFilterCoefficient(filterCoefficient BME680IIRFilterCoefficient) error {

	// Unset spi_3w_en<0> as this should always be 0 for the device being used with i2c
	// Set filter<2:0> to the specified filter coefficient
	return d.connection.WriteByteData(bme680RegisterConfig, (uint8(filterCoefficient) << 2))
}

func (d *BME680Driver) setupGasSensorConfiguration() error {

	// Unset nb_conv<3:0> to be zero to use device default settings for the gas sensor
	// Set run_gas<4> to enable gas sensor measurements
	return d.connection.WriteByteData(bme680RegisterCtrlGasOne, 0x10)
}

func (d *BME680Driver) UpdateSensorPowerMode(mode BME680SensorPowerMode) error {

	currentCtrlMeasurementVal, err := d.connection.ReadByteData(bme680RegisterCtrlMeasurement)
	if err != nil {
		return err
	}

	// Set mode<1:0> to mode
	// Preserve the value of osrs_t<2:0> and osrs_p<2:0>
	newCtrlMeasurementVal := (currentCtrlMeasurementVal & 0xFC) | uint8(mode)

	return d.connection.WriteByteData(bme680RegisterCtrlMeasurement, newCtrlMeasurementVal)
}

func (d *BME680Driver) GetCalibrationConstants() ([]byte, error) {
	c, err := d.read(bme680RegisterCoeffAddr1, 25)
	if err != nil {
		return nil, err
	}

	c2, err := d.read(bme680RegisterCoeffAddr2, 16)
	if err != nil {
		return nil, err
	}
	c = append(c, c2...)

	rawResHeatRange, err := d.connection.ReadByteData(bme680RegisterResistanceHeatRange)
	if err != nil {
		return nil, err
	}

	d.CalibrationCoefficients.resHeatRange = (rawResHeatRange & 0x30) >> 4

	rawSoftwareError, err := d.connection.ReadByteData(bme680RegisterRangeSoftwareError)
	if err != nil {
		return nil, err
	}

	d.CalibrationCoefficients.softwareError = (rawSoftwareError & 0x0f) >> 4

	setTemperatureCalibrationCoefficients(c, d.CalibrationCoefficients)
	setGasCalibrationCoefficients(c, d.CalibrationCoefficients)

	return c, nil
}

func setTemperatureCalibrationCoefficients(coeff []byte, s *BME680CalibrationCoefficients) {
	s.t1 = uint16(coeff[bme680T1MSBCalibrationReg])<<8 | uint16(coeff[bme680T1LSBCalibrationReg])
	s.t2 = int16(uint16(coeff[bme680T2MSBCalibrationReg])<<8 | uint16(coeff[bme680T2LSBCalibrationReg]))
	s.t3 = int8(coeff[bme680T3CalibrationReg])
}

func setGasCalibrationCoefficients(coeff []byte, s *BME680CalibrationCoefficients) {
	s.gh1 = int8(coeff[bme680GH1CalibrationReg])
	s.gh2 = int16(uint16(coeff[bme680GH2MSBCalibrationReg])<<8 | uint16(coeff[bme680GH2LSBCalibrationReg]))
	s.gh3 = int8(coeff[bme680GH3CalibrationReg])
}

func (d *BME680Driver) GetGasResistance() (uint32, error) {
	data, err := d.read(bme680RegisterGasResistanceMSB, 2)
	if err != nil {
		return 0, err
	}

	gasResistanceAdc := uint16(data[0]) | uint16(data[1]>>6)
	gasRange := uint8(data[1] & 0x0F)
	fmt.Printf("\n Gas Stuff: %+v %+v %+v \n", gasRange, gasResistanceAdc, d.CalibrationCoefficients.softwareError)

	var var1, var2, var3 int64

	var1 = int64((1340+(5*int64(d.CalibrationCoefficients.softwareError)))*(int64(bme680GasResistanceLookupTable1[gasRange]))) >> 16
	var2 = (int64(gasResistanceAdc) << 15) - (int64(16777216)) + var1
	var3 = (int64(bme680GasResistanceLookupTable2[gasRange]) * var1) >> 9
	return (uint32)((var3 + (int64(var2))>>1) / int64(var2)), nil
}

func (d *BME680Driver) GetTemperature() (int16, error) {
	data, err := d.read(bme680RegisterTempMSB, 3)
	if err != nil {
		return 0, err
	}

	tempAdc := (uint32(data[0]) * 4096) | (uint32(data[1]) * 16) | (uint32(data[2]) / 16)

	var tFine, var1, var2, var3 int32
	var1 = int32(tempAdc>>3) - (int32(d.CalibrationCoefficients.t1) << 1)
	var2 = (var1 * int32(d.CalibrationCoefficients.t2)) >> 11
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12
	var3 = ((var3) * (int32(d.CalibrationCoefficients.t3) << 4)) >> 14
	tFine = int32(var2 + var3)
	return int16(((tFine*5)+128)>>8) / 100, nil
}

// func (d *BME680Driver) GetGasResistance() (uint32, error) {
// 	data, err := d.read(bme680RegisterTempMSB, 3)
// 	if err != nil {
// 		return 0, err
// 	}

// 	// var var1, var2, var3 int64

// 	// var1 = ((1340 + (5 * self._sw_err)) * (_LOOKUP_TABLE_1[self._gas_range])) / 65536
// 	// var2 = ((self._adc_gas * 32768) - 16777216) + var1
// 	// var3 = (_LOOKUP_TABLE_2[self._gas_range] * var1) / 512
// 	// calc_gas_res = (var3 + (var2 / 2)) / var2

// 	// var1 = int64(((1340 + (5 * (int64_t) dev->calib.range_sw_err)) * ((int64_t) lookupTable1[gas_range])) >> 16)
// 	// var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
// 	// var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
// 	// calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

// 	// tempAdc := (uint32(data[0]) * 4096) | (uint32(data[1]) * 16) | (uint32(data[2]) / 16)

// 	// var tFine, var1, var2, var3 int32
// 	// var1 = int32(tempAdc>>3) - (int32(d.CalibrationCoefficients.t1) << 1)
// 	// var2 = (var1 * int32(d.CalibrationCoefficients.t2)) >> 11
// 	// var3 = ((var1 >> 1) * (var1 >> 1)) >> 12
// 	// var3 = ((var3) * (int32(d.CalibrationCoefficients.t3) << 4)) >> 14
// 	// tFine = int32(var2 + var3)
// 	// return int16(((tFine*5)+128)>>8) / 100, nil
// }

func (d *BME680Driver) read(address byte, n int) ([]byte, error) {
	if _, err := d.connection.Write([]byte{address}); err != nil {
		return nil, err
	}
	buf := make([]byte, n)
	bytesRead, err := d.connection.Read(buf)
	if bytesRead != n || err != nil {
		return nil, err
	}
	return buf, nil
}

// func (d *BME680Driver) getCalibrationCoefficients()

// // Humidity returns the current humidity in percentage of relative humidity
// func (d *BME280Driver) Humidity() (humidity float32, err error) {
// 	var rawH uint32
// 	if rawH, err = d.rawHumidity(); err != nil {
// 		return 0.0, err
// 	}
// 	humidity = d.calculateHumidity(rawH)
// 	return
// }

// // read the humidity calibration coefficients.
// func (d *BME280Driver) initHumidity() (err error) {
// 	var coefficients []byte
// 	if coefficients, err = d.read(bme280RegisterCalibDigH1, 1); err != nil {
// 		return err
// 	}
// 	buf := bytes.NewBuffer(coefficients)
// 	binary.Read(buf, binary.BigEndian, &d.hc.h1)

// 	if coefficients, err = d.read(bme280RegisterCalibDigH2LSB, 7); err != nil {
// 		return err
// 	}
// 	buf = bytes.NewBuffer(coefficients)

// 	// H4 and H5 laid out strangely on the bme280
// 	var addrE4 byte
// 	var addrE5 byte
// 	var addrE6 byte

// 	binary.Read(buf, binary.LittleEndian, &d.hc.h2) // E1 ...
// 	binary.Read(buf, binary.BigEndian, &d.hc.h3)    // E3
// 	binary.Read(buf, binary.BigEndian, &addrE4)     // E4
// 	binary.Read(buf, binary.BigEndian, &addrE5)     // E5
// 	binary.Read(buf, binary.BigEndian, &addrE6)     // E6
// 	binary.Read(buf, binary.BigEndian, &d.hc.h6)    // ... E7

// 	d.hc.h4 = 0 + (int16(addrE4) << 4) | (int16(addrE5 & 0x0F))
// 	d.hc.h5 = 0 + (int16(addrE6) << 4) | (int16(addrE5) >> 4)

// 	d.connection.WriteByteData(bme280RegisterControlHumidity, 0x3F)

// 	// The 'ctrl_hum' register sets the humidity data acquisition options of
// 	// the device. Changes to this register only become effective after a write
// 	// operation to 'ctrl_meas'. Read the current value in, then write it back
// 	var cmr uint8
// 	cmr, err = d.connection.ReadByteData(bmp280RegisterControl)
// 	if err == nil {
// 		err = d.connection.WriteByteData(bmp280RegisterControl, cmr)
// 	}
// 	return err
// }

// func (d *BME280Driver) rawHumidity() (uint32, error) {
// 	ret, err := d.read(bme280RegisterHumidityMSB, 2)
// 	if err != nil {
// 		return 0, err
// 	}
// 	if ret[0] == 0x80 && ret[1] == 0x00 {
// 		return 0, errors.New("Humidity disabled")
// 	}
// 	buf := bytes.NewBuffer(ret)
// 	var rawH uint16
// 	binary.Read(buf, binary.BigEndian, &rawH)
// 	return uint32(rawH), nil
// }

// // Adapted from https://github.com/BoschSensortec/BME280_driver/blob/master/bme280.c
// // function bme280_compensate_humidity_double(s32 v_uncom_humidity_s32)
// func (d *BME280Driver) calculateHumidity(rawH uint32) float32 {
// 	var rawT int32
// 	var err error
// 	var h float32

// 	rawT, err = d.rawTemp()
// 	if err != nil {
// 		return 0
// 	}

// 	_, tFine := d.calculateTemp(rawT)
// 	h = float32(tFine) - 76800

// 	if h == 0 {
// 		return 0 // TODO err is 'invalid data' from Bosch - include errors or not?
// 	}

// 	x := float32(rawH) - (float32(d.hc.h4)*64.0 +
// 		(float32(d.hc.h5) / 16384.0 * h))

// 	y := float32(d.hc.h2) / 65536.0 *
// 		(1.0 + float32(d.hc.h6)/67108864.0*h*
// 			(1.0+float32(d.hc.h3)/67108864.0*h))

// 	h = x * y
// 	h = h * (1 - float32(d.hc.h1)*h/524288)
// 	return h
// }
