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
bool isDhw_present;
bool isControl_type;
bool isCooling_present;
bool isDhw_tank_present;
bool isPump_control_present;
bool isCh2_present;

OpenTherm ot(inPin, outPin, false);

ICACHE_RAM_ATTR void handleInterrupt() {
	ot.handleInterrupt();
}

class OpenthermComponent: public PollingComponent {
private:
  const char *TAG = "opentherm_component";
  OpenthermFloatOutput *pid_output_; 
public:
  Switch *thermostatSwitch = new OpenthermSwitch();
  Sensor *external_temperature_sensor = new Sensor();
  Sensor *return_temperature_sensor = new Sensor();
  Sensor *boiler_temperature = new Sensor();
  Sensor *pressure_sensor = new Sensor();
  Sensor *modulation_sensor = new Sensor();
  Sensor *heating_target_temperature_sensor = new Sensor();
  Sensor *error_sensor = new Sensor();
  OpenthermClimate *hotWaterClimate = new OpenthermClimate();
  OpenthermClimate *heatingWaterClimate = new OpenthermClimate();
  BinarySensor *flame = new OpenthermBinarySensor();
  BinarySensor *fault = new OpenthermBinarySensor();
  BinarySensor *diagnostic = new OpenthermBinarySensor();
  
  BinarySensor *service_required = new OpenthermBinarySensor();
  BinarySensor *lockout_reset = new OpenthermBinarySensor();
  BinarySensor *low_water_pressure = new OpenthermBinarySensor();
  BinarySensor *gas_fault = new OpenthermBinarySensor();
  BinarySensor *air_fault = new OpenthermBinarySensor();
  BinarySensor *water_overtemp = new OpenthermBinarySensor();
  
  BinarySensor *dhw_present = new OpenthermBinarySensor();
  BinarySensor *control_type = new OpenthermBinarySensor();
  BinarySensor *cooling_present = new OpenthermBinarySensor();
  BinarySensor *dhw_tank_present = new OpenthermBinarySensor();
  BinarySensor *pump_control_present = new OpenthermBinarySensor();
  BinarySensor *ch2_present = new OpenthermBinarySensor();


  // Set 3 sec. to give time to read all sensors (and not appear in HA as not available)
  OpenthermComponent(): PollingComponent(3000) {
  }

  void set_pid_output(OpenthermFloatOutput *pid_output) { pid_output_ = pid_output; }


  void setup() override {
    // This will be called once to set up the component
    // think of it as the setup() call in Arduino
      ESP_LOGD("opentherm_component", "Setup");
      
      unsigned long request3 = ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::SConfigSMemberIDcode, 0xFFFF);
      unsigned long respons3 = ot.sendRequest(request3);
      uint8_t SlaveMemberIDcode = respons3 >> 0 & 0xFF;
      uint8_t flags = (respons3 & 0xFFFF) >> 8 & 0xFF;
      
      bool isDhw_present = flags & 0x01;
      bool isControl_type = flags & 0x02;
      bool isCooling_present = flags & 0x04;
      bool isDhw_tank_present = flags & 0x08;
      bool isPump_control_present = flags & 0x10;
      bool isCh2_present = flags & 0x20;
      
      ESP_LOGD ("opentherm_component", "SConfigSMemberIDcode %X", request3 );
   
      unsigned long request2 = ot.buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::MConfigMMemberIDcode, SlaveMemberIDcode);
      unsigned long respons2 = ot.sendRequest(request2);
      ESP_LOGD ("opentherm_component", "MConfigMMemberIDcode %X",  request2 ); 
      
      unsigned long request127 = ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::SlaveVersion, 0);
      ESP_LOGD ("opentherm_component", "SlaveVersion %X",  request127 ); 
      
      unsigned long request126 = ot.buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::MasterVersion, 0x013F);
      unsigned long respons126 = ot.sendRequest(request126);
      ESP_LOGD ("opentherm_component", "MasterVersion %X",  request126 ); 
      
      ot.begin(handleInterrupt);

      thermostatSwitch->add_on_state_callback([=](bool state) -> void {
        ESP_LOGD ("opentherm_component", "termostatSwitch_on_state_callback %d", state);    
      });

      // Adjust HeatingWaterClimate depending on PID
      heatingWaterClimate->set_supports_heat_cool_mode(this->pid_output_ != nullptr);
      // heatingWaterClimate->set_supports_two_point_target_temperature(this->pid_output_ != nullptr);

      hotWaterClimate->set_temperature_settings(5, 6, 0);
      heatingWaterClimate->set_temperature_settings(0, 0, 30);
      hotWaterClimate->setup();
      heatingWaterClimate->setup();
  }
  float getExternalTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Toutside, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
  }

  float getReturnTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
  }
  
  float getHotWaterTemperature() {
      unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tdhw, 0));
      return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
  }

  bool setHotWaterTemperature(float temperature) {
	    unsigned int data = ot.temperatureToData(temperature);
      unsigned long request = ot.buildRequest(OpenThermRequestType::WRITE, OpenThermMessageID::TdhwSet, data);
      unsigned long response = ot.sendRequest(request);
      return ot.isValidResponse(response);
  }

  float getModulation() {
    unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
  }

  float getPressure() {
    unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
  }
  int getASFflags() {
    unsigned long response = ot.sendRequest(ot.buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0));
    return ot.isValidResponse(response) ? ot.getFloat(response) : -1;
    uint8_t getASFflags = (response & 0xFFFF) >> 8 & 0xFF;
  }
 
    
  void update() override {

    ESP_LOGD("opentherm_component", "update heatingWaterClimate: %i", heatingWaterClimate->mode);
    ESP_LOGD("opentherm_component", "update hotWaterClimate: %i", hotWaterClimate->mode);

    bool enableCentralHeating = heatingWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT;
    bool enableHotWater = hotWaterClimate->mode == ClimateMode::CLIMATE_MODE_HEAT;
    bool enableCooling = false; // this boiler is for heating only

    
    //Set/Get Boiler Status
    auto response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    bool isFlameOn = ot.isFlameOn(response);
    bool isFault = ot.isFault(response);
    bool isDiagnostic = ot.isDiagnostic(response);
    bool isCentralHeatingActive = ot.isCentralHeatingActive(response);
    bool isHotWaterActive = ot.isHotWaterActive(response);
    
    int ASFflags = getASFflags();
    bool isService_required = ASFflags & 0x01;
    bool isLockout_reset = ASFflags & 0x02;
    bool isLow_water_pressure = ASFflags & 0x04;
    bool isGas_fault = ASFflags & 0x08;
    bool isAir_fault = ASFflags & 0x10;
    bool isWater_overtemp = ASFflags & 0x20;
    
    float return_temperature = getReturnTemperature();
    float hotWater_temperature = getHotWaterTemperature();
    unsigned char isError = ot.getFault();

    // Set temperature depending on room thermostat
    float heating_target_temperature;
    if (this->pid_output_ != nullptr) {
      float pid_output = pid_output_->get_state();
      if (pid_output == 0.0f) {
        heating_target_temperature = 0.0f;
      }
      else {
        heating_target_temperature =  pid_output * (heatingWaterClimate->target_temperature_high - heatingWaterClimate->target_temperature_low) 
        + heatingWaterClimate->target_temperature_low;      
      }
      ESP_LOGD("opentherm_component", "setBoilerTemperature  at %f °C (from PID Output)", heating_target_temperature);
    }
    else if (thermostatSwitch->state) {
      heating_target_temperature = heatingWaterClimate->target_temperature;
      ESP_LOGD("opentherm_component", "setBoilerTemperature  at %f °C (from heating water climate)", heating_target_temperature);
    }
    else {
      heating_target_temperature = 0.0;
      ESP_LOGD("opentherm_component", "setBoilerTemperature at %f °C (default low value)", heating_target_temperature);
    }
    ot.setBoilerTemperature(heating_target_temperature);

    // Set hot water temperature
    setHotWaterTemperature(hotWaterClimate->target_temperature);

    float boilerTemperature = ot.getBoilerTemperature();
    float ext_temperature = getExternalTemperature();
    float pressure = getPressure();
    float modulation = getModulation();
  

    // Publish sensor values
    flame->publish_state(isFlameOn); 
    fault->publish_state(isFault);
    diagnostic->publish_state(isDiagnostic);
    
    service_required->publish_state(isService_required); 
    lockout_reset->publish_state(isLockout_reset);
    low_water_pressure->publish_state(isLow_water_pressure);
    gas_fault->publish_state(isGas_fault); 
    air_fault->publish_state(isAir_fault);
    water_overtemp->publish_state(isWater_overtemp);
    
    dhw_present->publish_state(isDhw_present); 
    control_type->publish_state(isControl_type);
    cooling_present->publish_state(isCooling_present);
    dhw_tank_present->publish_state(isDhw_tank_present); 
    pump_control_present->publish_state(isPump_control_present);
    ch2_present->publish_state(isCh2_present);
    
    external_temperature_sensor->publish_state(ext_temperature);
    return_temperature_sensor->publish_state(return_temperature);
    boiler_temperature->publish_state(boilerTemperature);
    pressure_sensor->publish_state(pressure);
    modulation_sensor->publish_state(modulation);
    error_sensor->publish_state(isError); 


    heating_target_temperature_sensor->publish_state(heating_target_temperature);

    // Publish status of thermostat that controls hot water
    hotWaterClimate->current_temperature = hotWater_temperature;
    hotWaterClimate->action = isHotWaterActive ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    hotWaterClimate->publish_state();
    
    // Publish status of thermostat that controls heating
    heatingWaterClimate->current_temperature = boilerTemperature;
    heatingWaterClimate->action = isCentralHeatingActive && isFlameOn ? ClimateAction::CLIMATE_ACTION_HEATING : ClimateAction::CLIMATE_ACTION_OFF;
    heatingWaterClimate->publish_state();
  }

};
