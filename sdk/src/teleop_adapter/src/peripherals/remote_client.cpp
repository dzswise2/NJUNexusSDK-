#include "teleop_adapter/peripherals/remote_client.h"
#include "httplib.h"
#include "nlohmann/json.hpp"
#include <thread>
#include <atomic>
#include <chrono>

using json = nlohmann::json;

namespace peripherals_remote {

class PeripheralsClient::Impl {
private:
    std::unique_ptr<httplib::Client> client_;
    std::string server_url_;
    bool connected_;
    
    // 监控线程
    std::thread analog_monitor_thread_;
    std::thread digital_monitor_thread_;
    std::atomic<bool> analog_monitoring_{false};
    std::atomic<bool> digital_monitoring_{false};
    AnalogCallback analog_callback_;
    DigitalCallback digital_callback_;
    
    bool sendRequest(const std::string& method, const std::string& path,
                    const json& body, json& response) {
        if (!client_) {
            return false;
        }
        
        if (method == "GET") {
            auto res = client_->Get(path);
            if (res && res->status == 200) {
                try {
                    response = json::parse(res->body);
                    return response.value("success", false);
                } catch (...) {
                    return false;
                }
            }
        } else if (method == "POST") {
            auto res = client_->Post(path, body.dump(), "application/json");
            if (res && res->status == 200) {
                try {
                    response = json::parse(res->body);
                    return response.value("success", false);
                } catch (...) {
                    return false;
                }
            }
        }
        
        return false;
    }
    
    template<typename T>
    bool getParam(const std::string& path, const std::string& param_name, T& value) {
        json response;
        if (!sendRequest("GET", path, json::object(), response)) {
            return false;
        }
        
        if (response.contains("data") && response["data"].contains(param_name)) {
            value = response["data"][param_name].get<T>();
            return true;
        }
        
        return false;
    }
    
    template<typename T>
    bool postParam(const std::string& path, const std::string& param_name, const T& value) {
        json body = {{param_name, value}};
        json response;
        return sendRequest("POST", path, body, response);
    }
    
public:
    Impl() : connected_(false) {}
    
    ~Impl() {
        disconnect();
    }
    
    bool connect(const std::string& server_url) {
        server_url_ = server_url;
        
        // 解析URL
        size_t protocol_pos = server_url.find("://");
        std::string host;
        int port = 80;
        
        if (protocol_pos != std::string::npos) {
            host = server_url.substr(protocol_pos + 3);
        } else {
            host = server_url;
        }
        
        size_t colon_pos = host.find(":");
        if (colon_pos != std::string::npos) {
            port = std::stoi(host.substr(colon_pos + 1));
            host = host.substr(0, colon_pos);
        }
        
        client_ = std::make_unique<httplib::Client>(host, port);
     // 增加超时时间
        client_->set_connection_timeout(5);   // 连接超时改为5秒
        client_->set_read_timeout(30);        // 读取超时改为30秒（BMS可能需要较长时间）
        client_->set_write_timeout(10);       // 写入超时10秒
        
        connected_ = healthCheck();
        return connected_;
    }
    
    void disconnect() {
        stopAnalogMonitor();
        stopDigitalMonitor();
        client_.reset();
        connected_ = false;
    }
    
    bool isConnected() const {
        return connected_;
    }
    
    bool healthCheck() {
        json response;
        return sendRequest("GET", "/api/health", json::object(), response);
    }
    
    bool getDeviceStatus(std::map<std::string, bool>& status) {
        json response;
        if (!sendRequest("GET", "/api/status", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data")) {
            for (auto& [key, value] : response["data"].items()) {
                status[key] = value.get<bool>();
            }
            return true;
        }
        
        return false;
    }
    
    // 模拟量接口
    bool readAnalogVoltage(int channel, float& voltage) {
        std::string path = "/api/analog/voltage?channel=" + std::to_string(channel);
        return getParam(path, "voltage", voltage);
    }
    
    bool readAnalogCurrent(int channel, float& current) {
        std::string path = "/api/analog/current?channel=" + std::to_string(channel);
        return getParam(path, "current", current);
    }
    
    bool readAnalogRaw(int channel, int& raw_value) {
        std::string path = "/api/analog/raw?channel=" + std::to_string(channel);
        return getParam(path, "raw_value", raw_value);
    }
    
    bool readAllAnalog(std::vector<AnalogInfo>& channels) {
        json response;
        if (!sendRequest("GET", "/api/analog/all", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data") && response["data"].contains("channels")) {
            channels.clear();
            for (auto& item : response["data"]["channels"]) {
                AnalogInfo info;
                info.channel = item["channel"];
                info.voltage = item["voltage"];
                channels.push_back(info);
            }
            return true;
        }
        
        return false;
    }
    
    bool setAnalogVoltage(int channel, float voltage) {
        return postParam("/api/analog/voltage", "channel", channel) &&
               postParam("/api/analog/voltage", "voltage", voltage);
    }
    
    bool setAnalogRaw(int channel, int raw_value) {
        return postParam("/api/analog/raw", "channel", channel) &&
               postParam("/api/analog/raw", "raw_value", raw_value);
    }
    
    // 数字IO接口
    bool readDigitalInput(int board, int port, int& value) {
        std::string path = "/api/digital/input?board=" + std::to_string(board) + 
                          "&port=" + std::to_string(port);
        return getParam(path, "value", value);
    }
    
    bool readDigitalInput(int channel, int& value) {
        std::string path = "/api/digital/input?channel=" + std::to_string(channel);
        return getParam(path, "value", value);
    }
    
    bool readAllDigitalInputs(std::vector<DigitalInfo>& inputs) {
        json response;
        if (!sendRequest("GET", "/api/digital/inputs", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data") && response["data"].contains("inputs")) {
            inputs.clear();
            for (auto& item : response["data"]["inputs"]) {
                DigitalInfo info;
                info.channel = item["channel"];
                info.value = item["value"];
                inputs.push_back(info);
            }
            return true;
        }
        
        return false;
    }
    
    bool setDigitalOutput(int board, int port, int value) {
        json body = {{"board", board}, {"port", port}, {"value", value}};
        json response;
        return sendRequest("POST", "/api/digital/output", body, response);
    }
    
    bool setDigitalOutput(int channel, int value) {
        json body = {{"channel", channel}, {"value", value}};
        json response;
        return sendRequest("POST", "/api/digital/output", body, response);
    }
    
    bool setSimulationMode(bool enable) {
        return postParam("/api/digital/simulation", "enable", enable);
    }
    
    // 云台接口
    bool movePTZ(uint8_t servo_id, float angle) {
        json body = {{"servo_id", servo_id}, {"angle", angle}};
        json response;
        return sendRequest("POST", "/api/ptz/move", body, response);
    }
    
    bool getPTZAngle(uint8_t servo_id, float& angle) {
        std::string path = "/api/ptz/angle?servo_id=" + std::to_string(servo_id);
        return getParam(path, "angle", angle);
    }
    
    bool stopPTZ(uint8_t servo_id) {
        return postParam("/api/ptz/stop", "servo_id", servo_id);
    }
    
    bool stopAllPTZ() {
        json response;
        return sendRequest("POST", "/api/ptz/stop_all", json::object(), response);
    }
    
    bool homePTZ() {
        json response;
        return sendRequest("POST", "/api/ptz/home", json::object(), response);
    }
    
    // 夹爪接口
    bool setGripperPosition(int position) {
        return postParam("/api/gripper/position", "position", position);
    }
    
    bool getGripperPosition(int& position) {
        return getParam("/api/gripper/position", "position", position);
    }
    
    bool enableGripper(bool enable) {
        return postParam("/api/gripper/enable", "enable", enable);
    }
    
    bool stopGripper() {
        json response;
        return sendRequest("POST", "/api/gripper/stop", json::object(), response);
    }
    
    bool getGripperStatus(GripperStatus& status) {
        json response;
        if (!sendRequest("GET", "/api/gripper/status", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data")) {
            status.ready = response["data"]["ready"];
            status.position_reached = response["data"]["position_reached"];
            status.torque_reached = response["data"]["torque_reached"];
            status.alarm = response["data"]["alarm"];
            return true;
        }
        
        return false;
    }
    
    bool clearGripperAlarm() {
        json response;
        return sendRequest("POST", "/api/gripper/clear_alarm", json::object(), response);
    }
    
    // BMS接口
    bool readBMSBasicInfo(BMSBasicInfo& info) {
        json response;
        if (!sendRequest("GET", "/api/bms/basic_info", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data")) {
            auto& data = response["data"];
            info.total_voltage = data["total_voltage"];
            info.current = data["current"];
            info.remaining_capacity = data["remaining_capacity"];
            info.nominal_capacity = data["nominal_capacity"];
            info.cycle_count = data["cycle_count"];
            info.rsoc = data["rsoc"];
            info.fet_control = data["fet_control"];
            info.protection_status = data["protection_status"];
            info.cell_count = data["cell_count"];
            
            if (data.contains("temperatures")) {
                for (auto& temp : data["temperatures"]) {
                    info.temperatures.push_back(temp);
                }
            }
            return true;
        }
        
        return false;
    }
    
    bool readBMSCellVoltages(BMSCellVoltageInfo& info) {
        json response;
        if (!sendRequest("GET", "/api/bms/cell_voltages", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data")) {
            auto& data = response["data"];
            info.cell_count = data["cell_count"];
            info.min_voltage = data["min_voltage"];
            info.max_voltage = data["max_voltage"];
            info.avg_voltage = data["avg_voltage"];
            info.min_voltage_cell = data["min_voltage_cell"];
            info.max_voltage_cell = data["max_voltage_cell"];
            info.voltage_diff = data["voltage_diff"];
            
            for (auto& volt : data["voltages"]) {
                info.voltages.push_back(volt);
            }
            return true;
        }
        
        return false;
    }
    
    bool setChargeMOS(bool on) {
        return postParam("/api/bms/charge_mos", "on", on);
    }
    
    bool setDischargeMOS(bool on) {
        return postParam("/api/bms/discharge_mos", "on", on);
    }
    
    bool getMOSStatus(bool& charge_on, bool& discharge_on) {
        json response;
        if (!sendRequest("GET", "/api/bms/mos_status", json::object(), response)) {
            return false;
        }
        
        if (response.contains("data")) {
            charge_on = response["data"]["charge_mos_on"];
            discharge_on = response["data"]["discharge_mos_on"];
            return true;
        }
        
        return false;
    }
    
    bool resetBMSCapacity() {
        json response;
        return sendRequest("POST", "/api/bms/reset_capacity", json::object(), response);
    }
    
    bool clearBMSProtection() {
        json response;
        return sendRequest("POST", "/api/bms/clear_protection", json::object(), response);
    }
    
    // 光源接口
    bool setLightBrightness(int channel, int brightness) {
        json body = {{"channel", channel}, {"brightness", brightness}};
        json response;
        return sendRequest("POST", "/api/light/brightness", body, response);
    }
    
    bool getLightBrightness(int channel, int& brightness) {
        std::string path = "/api/light/brightness?channel=" + std::to_string(channel);
        return getParam(path, "brightness", brightness);
    }
    
    bool setLightOnOff(int channel, bool on) {
        json body = {{"channel", channel}, {"on", on}};
        json response;
        return sendRequest("POST", "/api/light/onoff", body, response);
    }
    
    bool saveLightParams() {
        json response;
        return sendRequest("POST", "/api/light/save", json::object(), response);
    }
    
    // 旋转吸盘接口
    bool setSuckerAngle(int angle) {
        return postParam("/api/sucker/angle", "angle", angle);
    }
    
    bool getSuckerAngle(int& angle) {
        return getParam("/api/sucker/angle", "angle", angle);
    }
    
    bool setSuckerDuty(int duty) {
        return postParam("/api/sucker/duty", "duty", duty);
    }
    
    bool getSuckerDuty(int& duty) {
        return getParam("/api/sucker/duty", "duty", duty);
    }
    
    bool homeSucker() {
        json response;
        return sendRequest("POST", "/api/sucker/home", json::object(), response);
    }
    
    // 监控功能
    void startAnalogMonitor(int interval_ms, AnalogCallback callback) {
        if (analog_monitoring_) return;
        
        analog_monitoring_ = true;
        analog_callback_ = callback;
        
        analog_monitor_thread_ = std::thread([this, interval_ms]() {
            while (analog_monitoring_) {
                for (int channel = 0; channel < 4; channel++) {
                    float voltage;
                    if (readAnalogVoltage(channel, voltage) && analog_callback_) {
                        analog_callback_(channel, voltage);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        });
    }
    
    void stopAnalogMonitor() {
        analog_monitoring_ = false;
        if (analog_monitor_thread_.joinable()) {
            analog_monitor_thread_.join();
        }
    }
    
    void startDigitalMonitor(int interval_ms, DigitalCallback callback) {
        if (digital_monitoring_) return;
        
        digital_monitoring_ = true;
        digital_callback_ = callback;
        
        digital_monitor_thread_ = std::thread([this, interval_ms]() {
            while (digital_monitoring_) {
                for (int channel = 0; channel < 16; channel++) {
                    int value;
                    if (readDigitalInput(channel, value) && digital_callback_) {
                        digital_callback_(channel, value);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
            }
        });
    }
    
    void stopDigitalMonitor() {
        digital_monitoring_ = false;
        if (digital_monitor_thread_.joinable()) {
            digital_monitor_thread_.join();
        }
    }
};

// Public接口实现
PeripheralsClient::PeripheralsClient() : pImpl_(std::make_unique<Impl>()) {}

PeripheralsClient::PeripheralsClient(const std::string& server_url) 
    : pImpl_(std::make_unique<Impl>()) {
    connect(server_url);
}

PeripheralsClient::~PeripheralsClient() = default;

bool PeripheralsClient::connect(const std::string& server_url) {
    return pImpl_->connect(server_url);
}

void PeripheralsClient::disconnect() {
    pImpl_->disconnect();
}

bool PeripheralsClient::isConnected() const {
    return pImpl_->isConnected();
}

bool PeripheralsClient::healthCheck() {
    return pImpl_->healthCheck();
}

bool PeripheralsClient::getDeviceStatus(std::map<std::string, bool>& status) {
    return pImpl_->getDeviceStatus(status);
}

bool PeripheralsClient::readAnalogVoltage(int channel, float& voltage) {
    return pImpl_->readAnalogVoltage(channel, voltage);
}

bool PeripheralsClient::readAnalogCurrent(int channel, float& current) {
    return pImpl_->readAnalogCurrent(channel, current);
}

bool PeripheralsClient::readAnalogRaw(int channel, int& raw_value) {
    return pImpl_->readAnalogRaw(channel, raw_value);
}

bool PeripheralsClient::readAllAnalog(std::vector<AnalogInfo>& channels) {
    return pImpl_->readAllAnalog(channels);
}

bool PeripheralsClient::setAnalogVoltage(int channel, float voltage) {
    return pImpl_->setAnalogVoltage(channel, voltage);
}

bool PeripheralsClient::setAnalogRaw(int channel, int raw_value) {
    return pImpl_->setAnalogRaw(channel, raw_value);
}

// ... 前面的代码保持不变 ...

bool PeripheralsClient::readDigitalInput(int board, int port, int& value) {
    return pImpl_->readDigitalInput(board, port, value);
}

bool PeripheralsClient::readDigitalInput(int channel, int& value) {
    return pImpl_->readDigitalInput(channel, value);
}

bool PeripheralsClient::readAllDigitalInputs(std::vector<DigitalInfo>& inputs) {
    return pImpl_->readAllDigitalInputs(inputs);
}

bool PeripheralsClient::setDigitalOutput(int board, int port, int value) {
    return pImpl_->setDigitalOutput(board, port, value);
}

bool PeripheralsClient::setDigitalOutput(int channel, int value) {
    return pImpl_->setDigitalOutput(channel, value);
}

bool PeripheralsClient::setSimulationMode(bool enable) {
    return pImpl_->setSimulationMode(enable);
}

bool PeripheralsClient::movePTZ(uint8_t servo_id, float angle) {
    return pImpl_->movePTZ(servo_id, angle);
}

bool PeripheralsClient::getPTZAngle(uint8_t servo_id, float& angle) {
    return pImpl_->getPTZAngle(servo_id, angle);
}

bool PeripheralsClient::stopPTZ(uint8_t servo_id) {
    return pImpl_->stopPTZ(servo_id);
}

bool PeripheralsClient::stopAllPTZ() {
    return pImpl_->stopAllPTZ();
}

bool PeripheralsClient::homePTZ() {
    return pImpl_->homePTZ();
}

bool PeripheralsClient::setGripperPosition(int position) {
    return pImpl_->setGripperPosition(position);
}

bool PeripheralsClient::getGripperPosition(int& position) {
    return pImpl_->getGripperPosition(position);
}

bool PeripheralsClient::enableGripper(bool enable) {
    return pImpl_->enableGripper(enable);
}

bool PeripheralsClient::stopGripper() {
    return pImpl_->stopGripper();
}

bool PeripheralsClient::getGripperStatus(GripperStatus& status) {
    return pImpl_->getGripperStatus(status);
}

bool PeripheralsClient::clearGripperAlarm() {
    return pImpl_->clearGripperAlarm();
}

bool PeripheralsClient::readBMSBasicInfo(BMSBasicInfo& info) {
    return pImpl_->readBMSBasicInfo(info);
}

bool PeripheralsClient::readBMSCellVoltages(BMSCellVoltageInfo& info) {
    return pImpl_->readBMSCellVoltages(info);
}

bool PeripheralsClient::setChargeMOS(bool on) {
    return pImpl_->setChargeMOS(on);
}

bool PeripheralsClient::setDischargeMOS(bool on) {
    return pImpl_->setDischargeMOS(on);
}

bool PeripheralsClient::getMOSStatus(bool& charge_on, bool& discharge_on) {
    return pImpl_->getMOSStatus(charge_on, discharge_on);
}

bool PeripheralsClient::resetBMSCapacity() {
    return pImpl_->resetBMSCapacity();
}

bool PeripheralsClient::clearBMSProtection() {
    return pImpl_->clearBMSProtection();
}

bool PeripheralsClient::setLightBrightness(int channel, int brightness) {
    return pImpl_->setLightBrightness(channel, brightness);
}

bool PeripheralsClient::getLightBrightness(int channel, int& brightness) {
    return pImpl_->getLightBrightness(channel, brightness);
}

bool PeripheralsClient::setLightOnOff(int channel, bool on) {
    return pImpl_->setLightOnOff(channel, on);
}

bool PeripheralsClient::saveLightParams() {
    return pImpl_->saveLightParams();
}

bool PeripheralsClient::setSuckerAngle(int angle) {
    return pImpl_->setSuckerAngle(angle);
}

bool PeripheralsClient::getSuckerAngle(int& angle) {
    return pImpl_->getSuckerAngle(angle);
}

bool PeripheralsClient::setSuckerDuty(int duty) {
    return pImpl_->setSuckerDuty(duty);
}

bool PeripheralsClient::getSuckerDuty(int& duty) {
    return pImpl_->getSuckerDuty(duty);
}

bool PeripheralsClient::homeSucker() {
    return pImpl_->homeSucker();
}

void PeripheralsClient::startAnalogMonitor(int interval_ms, AnalogCallback callback) {
    pImpl_->startAnalogMonitor(interval_ms, callback);
}

void PeripheralsClient::stopAnalogMonitor() {
    pImpl_->stopAnalogMonitor();
}

void PeripheralsClient::startDigitalMonitor(int interval_ms, DigitalCallback callback) {
    pImpl_->startDigitalMonitor(interval_ms, callback);
}

void PeripheralsClient::stopDigitalMonitor() {
    pImpl_->stopDigitalMonitor();
}

} // namespace peripherals_remote
