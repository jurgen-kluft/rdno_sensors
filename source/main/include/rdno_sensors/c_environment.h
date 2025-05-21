#ifndef __RDNO_SENSORS_ENVIRONMENT_H__
#define __RDNO_SENSORS_ENVIRONMENT_H__

namespace nenvironment
{
    /// Temperature unit enumeration.
    enum TempUnit
    {
        TempUnit_Celsius,
        TempUnit_Fahrenheit
    };

    /// Altitude unit enumeration.
    enum AltitudeUnit
    {
        AltitudeUnit_Meters,
        AltitudeUnit_Feet
    };

    /// Calculate the altitude based on the pressure and temperature
    /// in temptUnit.
    /// @param pressure at the station in any units.
    /// @param altUnit meters or feet. default=AltitudeUnit_Meters
    /// @param referencePressure (usually pressure on MSL)
    ///          in the same units as pressure. default=1013.25hPa (ISA)
    /// @param outdoorTemp temperature at the station in tempUnit
    ///          default=15°C (ISA)
    /// @param temptUnit in °C or °F. default=TempUnit_Celsius
    /// @return Calculated Altitude in altUnit.
    float Altitude(float pressure, AltitudeUnit altUnit = AltitudeUnit_Meters,
                   float    referencePressure = 1013.25,  // [hPa] ....ISA value
                   float    outdoorTemp       = 15,       // [°C] .... ISA value
                   TempUnit tempUnit          = TempUnit_Celsius);

    /// Calculate the heatindex based on the humidity and temperature
    /// in tempUnit.
    /// The formula based on the Heat Index Equation of the US National Weather Service
    /// http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
    /// @param temperature in tempUnit
    /// @param humidity in percentage
    /// @param temptUnit in °C or °F. default=TempUnit_Celsius
    /// @return Calculated heatindex as float in TempUnit
    float HeatIndex(float temperature, float humidity, TempUnit tempUnit = TempUnit_Celsius);

    /// Calculate the absolute humidity based on the relative humidity and temperature
    /// in tempUnit.
    /// the formula does work for values between -30°C and 35°C with 0.1°C precision
    /// @param temperature in tempUnit
    /// @param humidity in percentage
    /// @param tempUnit in °C. default=TempUnit_Celsius
    /// @return Calculated absolute humidity in grams/m³
    float AbsoluteHumidity(float temperature, float humidity, TempUnit tempUnit);

    /// Convert current pressure to equivalent sea-level pressure.
    /// @param altitude in altUnit.
    /// @param temp in tempUnit.
    /// @param pressure at the station in any units.
    /// @param altUnit meters or feet. default=AltitudeUnit_Meters
    /// @param tempUnit in °C or °F. default=TempUnit_Celsius
    /// @return Equivalent pressure at sea level. The input pressure
    ///          unit will determine the output
    ///          pressure unit.
    float EquivalentSeaLevelPressure(float altitude, float temp, float pres, AltitudeUnit altUnit = AltitudeUnit_Meters, TempUnit tempUnit = TempUnit_Celsius);

    /// Calculate the dew point based on the temperature in tempUnit
    /// and humidity.
    /// @param temp in tempUnit.
    /// @param hum in %.
    /// @param temptUnit in °C or °F. default=TempUnit_Celsius
    float DewPoint(float temp, float hum, TempUnit tempUnit = TempUnit_Celsius);

}  // namespace nenvironment

#endif  // TG_ENVIRONMENT_CALCULATIONS_H
