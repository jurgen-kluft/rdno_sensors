package rdno_sensors

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	cunittest "github.com/jurgen-kluft/cunittest/package"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_sensors"
)

// rdno_sensors is a  package for Arduino projects that holds a
// couple of sensor objects, namely:
// - BME280 (temperature, humidity, pressure)
// - BH1750 (light sensor)
// - Sensirion/SCD41 (carbon dioxide sensor, temperature, humidity)
func GetPackage() *denv.Package {
	name := repo_name

	// dependencies
	cunittestpkg := cunittest.GetPackage()
	corepkg := rdno_core.GetPackage()

	// main package
	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)
	mainpkg.AddPackage(cunittestpkg)

	// TODO, make a library per sensor ?

	// main library
	mainlib := denv.SetupCppLibProject(mainpkg, name)
	mainlib.AddDependencies(corepkg.GetMainLib())

	// test library
	testlib := denv.SetupCppTestLibProject(mainpkg, name)
	testlib.AddDependencies(corepkg.GetTestLib())

	// unittest project
	maintest := denv.SetupCppTestProject(mainpkg, name)
	maintest.AddDependencies(cunittestpkg.GetMainLib())
	maintest.AddDependency(testlib)

	mainpkg.AddMainLib(mainlib)
	mainpkg.AddTestLib(testlib)
	mainpkg.AddUnittest(maintest)
	return mainpkg
}
