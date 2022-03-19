#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "OpenTherm.h"
#include "opentherm_switch.h"
#include "opentherm_climate.h"
#include "opentherm_binary.h"
#include "opentherm_output.h"

// Pins to OpenTherm Adapter
int inPin = D2; 
int outPin = D1;
OpenTherm ot(inPin, outPin, false);

IRAM_ATTR void handleInterrupt() {
	ot.handleInterrupt();
}

class OpenthermComponent: public PollingComponent {
private:
  const char *TAG = "opentherm_component";
  OpenthermFloatOutput *thermostat_modulation_; 
public:
  Switch *modulating_thermostat_switch = new OpenthermSwitch();
  Sensor *outside_temperature_sensor = new Sensor();
  Sensor *return_temperature_sensor = new Sensor();
  Sensor *boiler_pressure_sensor = new Sensor();
  Sensor *boiler_modulation_sensor = new Sensor();
  Sensor *central_heating_actual_temperature_sensor = new Sensor();
  Sensor *central_heating_target_temperature_sensor = new Sensor();
  OpenthermClimate *domestic_hot_water_climate = new OpenthermClimate();
  OpenthermClimate *central_heating_climate = new OpenthermClimate();
  BinarySensor *boiler_flame_sensor = new OpenthermBinarySensor();
  
  // Set 3 sec. to give time to read all sensors (and not appear in HA as not available)
  OpenthermComponent(): PollingComponent(3000) {
  }

  void setThermostatModulation(OpenthermFloatOutput *thermostat_modulation) { thermostat_modulation_ = thermostat_modulation; }


  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
      ESP_LOGD("opentherm_component", "Setup");

      ot.begin(handleInterrupt);

      modulating_thermostat_switch->add_on_state_callback([=](bool state) -> void {
        ESP_LOGD ("opentherm_component", "modulating_thermostat_switch_on_state_callback %d", state);    
      });

      // Adjust HeatingWaterClimate depending on PID
      // heatingWaterClimate->set_supports_heat_cool_mode(this->pid_output_ != nullptr);
      
      domestic_hot_water_climate->set_temperature_settings(5, 6, 5.5);
      domestic_hot_water_climate->setup();
	  
      central_heating_climate->set_temperature_settings(19.5, 20.5, 20);
      central_heating_climate->set_supports_two_point_target_temperature(this->thermostat_modulation_ != nullptr);
      central_heating_climate->setup();
  }
  float getOutsideTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Toutside, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }

  float getReturnTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }
  
  float getDomesticHotWaterTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tdhw, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }

  bool setDomesticHotWaterTemperature(float temperature) {
      unsigned int data = ot.temperatureToData(temperature);
      unsigned long request = ot.buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::TdhwSet, data);
      unsigned long response = ot.sendRequest(request);
      return ot.isValidResponse(response);
  }

  float getRelativeModulationLevel() {
    unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }

  float getPressure() {
    unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return ot.isValidResponse(response) ? ot.getFloat(response) : NAN;
  }

  void update() override {

    ESP_LOGD("opentherm_component", "update central_heating_climate: %i", central_heating_climate->mode);
    ESP_LOGD("opentherm_component", "update domestic_hot_water_climate: %i", domestic_hot_water_climate->mode);
    
    bool enable_central_heating = central_heating_climate->mode == ClimateMode::CLIMATE_MODE_HEAT;
    bool enable_domestic_hot_water = domestic_hot_water_climate->mode == ClimateMode::CLIMATE_MODE_HEAT;
    bool enable_cooling = false; // this boiler is for heating only

    
    //Set/Get Boiler Status
    auto response = ot.setBoilerStatus(enable_central_heating, enable_domestic_hot_water, enable_cooling);
    bool is_flame_on = ot.isFlameOn(response);
    bool is_central_heating_active = ot.isCentralHeatingActive(response);
    bool is_hot_water_active = ot.isHotWaterActive(response);
    float return_temperature = getReturnTemperature();
    float outside_temperature = getOutsideTemperature();
    float boiler_pressure = getPressure();
    float boiler_modulation = getRelativeModulationLevel();
    float central_heating_actual_temperature = ot.getBoilerTemperature();
    float domestic_hot_water_temperature = getDomesticHotWaterTemperature();


    // Set temperature depending on room thermostat
    float central_heating_target_temperature = NAN;
    if (this->thermostat_modulation_ != nullptr) {
      float thermostat_modulation = thermostat_modulation_->get_state();
      if (thermostat_modulation == 0.0f) {
        central_heating_target_temperature = 10.0f;
      }
      else {
        central_heating_target_temperature =  thermostat_modulation * (central_heating_climate->target_temperature_high - central_heating_climate->target_temperature_low) 
        + central_heating_climate->target_temperature_low;
      }
      ESP_LOGD("opentherm_component", "setBoilerTemperature  at %f °C (from PID Output)", central_heating_target_temperature);
    }
    else {
      central_heating_target_temperature = central_heating_climate->target_temperature;
      ESP_LOGD("opentherm_component", "setBoilerTemperature  at %f °C (from heating water climate)", central_heating_target_temperature);
    }

    ot.setBoilerTemperature(central_heating_target_temperature);

    // Set hot water temperature
    setDomesticHotWaterTemperature(domestic_hot_water_climate->target_temperature);


    // Publish sensor values
    boiler_flame_sensor->publish_state(is_flame_on); 
    outside_temperature_sensor->publish_state(outside_temperature);
    return_temperature_sensor->publish_state(return_temperature);
    boiler_pressure_sensor->publish_state(boiler_pressure);
    boiler_modulation_sensor->publish_state(boiler_modulation);
    central_heating_actual_temperature_sensor->publish_state(central_heating_actual_temperature);
    central_heating_target_temperature_sensor->publish_state(central_heating_target_temperature);

    // Publish status of thermostat that controls hot water
    domestic_hot_water_climate->current_temperature = domestic_hot_water_temperature;
    domestic_hot_water_climate->action = is_hot_water_active ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    domestic_hot_water_climate->publish_state();
    
    // Publish status of thermostat that controls heating
    central_heating_climate->current_temperature = central_heating_actual_temperature;
    central_heating_climate->action = is_central_heating_active && is_flame_on ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    central_heating_climate->publish_state();
  }

};
