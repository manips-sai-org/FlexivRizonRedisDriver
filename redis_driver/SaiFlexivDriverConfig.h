#pragma once

#include "tinyxml2.h"
#include <stdexcept>
#include <string>

using std::runtime_error;

namespace Sai {
namespace Flexiv {

enum class RobotType { RIZON_4, RIZON_4S };

struct DriverConfig {
    std::string robot_name;
    std::string serial_number;
    RobotType robot_type;
    std::string force_sensor_link_name = "flange";
    std::string redis_prefix = "sai";
    bool verbose = true;
};

DriverConfig loadConfig(const std::string &config_file) {
    DriverConfig config;

    // load config file
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(config_file.c_str()) != tinyxml2::XML_SUCCESS) {
        throw runtime_error("Could not load driver config file: " +
                            config_file);
    }

    // parse driver config
    tinyxml2::XMLElement *driver_xml =
        doc.FirstChildElement("saiFlexivDriverConfig");
    if (driver_xml == nullptr) {
        throw runtime_error(
            "No 'saiFlexivDriverConfig' element found in driver config file: " +
            config_file);
    }

    // robot name
    if (!driver_xml->Attribute("robotName")) {
        throw runtime_error(
            "No 'robotName' attribute found in driver config file: " +
            config_file);
    }
    config.robot_name = driver_xml->Attribute("robotName");

    // robot ip
    if (!driver_xml->Attribute("serialNumber")) {
        throw runtime_error(
            "No 'serialNumber' attribute found in driver config file: " +
            config_file);
    }
    config.serial_number = driver_xml->Attribute("serialNumber");

    // redis prefix
    if (driver_xml->Attribute("redisPrefix")) {
        config.redis_prefix = driver_xml->Attribute("redisPrefix");
    }

    // robot type
    if (!driver_xml->Attribute("robotType")) {
        throw runtime_error(
            "No 'robotType' attribute found in driver config file: " +
            config_file);
    }
    std::string robot_type_str = driver_xml->Attribute("robotType");
    if (robot_type_str == "Rizon4") {
        config.robot_type = RobotType::RIZON_4;
    } else if (robot_type_str == "Rizon4s") {
        config.robot_type = RobotType::RIZON_4S;
    } else {
        throw runtime_error("Unknown robot type: " + robot_type_str +
                            "\nsupported types are: Rizon4, Rizon4s");
    }

    // force sensor link name
    if (driver_xml->Attribute("forceSensorLinkName")) {
        config.force_sensor_link_name =
            driver_xml->Attribute("forceSensorLinkName");
    }

    // verbose
    if (driver_xml->Attribute("verbose")) {
        config.verbose = driver_xml->BoolAttribute("verbose");
    }

    return config;
}

} // namespace Flexiv
} // namespace Sai