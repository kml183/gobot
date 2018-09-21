// +build example
//
// Do not build by default.

package main

import (
	"log"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/drivers/i2c"
	"gobot.io/x/gobot/platforms/raspi"
)

func bme680Runner(a *i2c.BME680Driver) (err error) {
	log.Println("Hello World")
	ts, _ := a.GetTemperature()
	s, _ := a.GetCalibrationConstants()
	gr, _ := a.GetGasResistance()
	log.Printf("%+v", s)
	log.Printf("\n\n Actual Temperature: %+v \n\n", ts)
	log.Printf("\n\n Gas Resistance in Ohms: %+v \n\n", gr)
	return nil
}

func main() {
	r := raspi.NewAdaptor()
	bme680 := i2c.NewBME680Driver(r)

	work := func() {
		gobot.Every(2*time.Second, func() {
			bme680Runner(bme680)
		})
	}

	robot := gobot.NewRobot("bme680Bot",
		[]gobot.Connection{r},
		[]gobot.Device{bme680},
		work,
	)

	robot.Start()
}
