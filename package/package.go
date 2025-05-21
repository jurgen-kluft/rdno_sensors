package rdno_sensors

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
)

// rdno_sensors is a  package for Arduino projects that holds a
// couple of sensor objects, namely:
// - BME280 (temperature, humidity, pressure)
// - BH1750 (light sensor)
// - Sensirion/SCD41 (carbon dioxide sensor, temperature, humidity)
func GetPackage() *denv.Package {
	corepkg := rdno_core.GetPackage()

	mainpkg := denv.NewPackage("rdno_sensors")
	mainpkg.AddPackage(corepkg)

	mainlib := denv.SetupCppLibProject("rdno_sensors", "github.com\\jurgen-kluft\\rdno_sensors")
	mainlib.AddDependencies(corepkg.GetMainLib()...)

	mainpkg.AddMainLib(mainlib)
	return mainpkg
}
