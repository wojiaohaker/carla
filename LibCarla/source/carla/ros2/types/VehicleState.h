// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file VehicleState.h
 * This header file contains the declaration of the described types in the IDL file.
 */

#ifndef _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_VEHICLESTATE_H_
#define _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_VEHICLESTATE_H_

#include "Header.h"

#include <fastrtps/utils/fixed_size_string.hpp>
#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(VehicleState_SOURCE)
#define VehicleState_DllAPI __declspec( dllexport )
#else
#define VehicleState_DllAPI __declspec( dllimport )
#endif
#else
#define VehicleState_DllAPI
#endif
#else
#define VehicleState_DllAPI
#endif

namespace eprosima {
namespace fastcdr {
class Cdr;
}
}

namespace vehicle_msgs {
    namespace msg {
        class VehicleState
        {
        public:
            eProsima_user_DllExport VehicleState();
            eProsima_user_DllExport ~VehicleState();
            eProsima_user_DllExport VehicleState(const VehicleState& x);
            eProsima_user_DllExport VehicleState(VehicleState&& x) noexcept;
            eProsima_user_DllExport VehicleState& operator =(const VehicleState& x);
            eProsima_user_DllExport VehicleState& operator =(VehicleState&& x) noexcept;
            eProsima_user_DllExport bool operator ==(const VehicleState& x) const;
            eProsima_user_DllExport bool operator !=(const VehicleState& x) const;

            eProsima_user_DllExport void header(const std_msgs::msg::Header& _header);
            eProsima_user_DllExport void header(std_msgs::msg::Header&& _header);
            eProsima_user_DllExport const std_msgs::msg::Header& header() const;
            eProsima_user_DllExport std_msgs::msg::Header& header();

            eProsima_user_DllExport void work_state(uint8_t _v);
            eProsima_user_DllExport uint8_t work_state() const;
            eProsima_user_DllExport uint8_t& work_state();

            eProsima_user_DllExport void work_mode(uint8_t _v);
            eProsima_user_DllExport uint8_t work_mode() const;
            eProsima_user_DllExport uint8_t& work_mode();

            eProsima_user_DllExport void control_model(uint8_t _v);
            eProsima_user_DllExport uint8_t control_model() const;
            eProsima_user_DllExport uint8_t& control_model();

            eProsima_user_DllExport void battery_capacity(uint8_t _v);
            eProsima_user_DllExport uint8_t battery_capacity() const;
            eProsima_user_DllExport uint8_t& battery_capacity();

            eProsima_user_DllExport void voltage(uint16_t _v);
            eProsima_user_DllExport uint16_t voltage() const;
            eProsima_user_DllExport uint16_t& voltage();

            eProsima_user_DllExport void current(uint16_t _v);
            eProsima_user_DllExport uint16_t current() const;
            eProsima_user_DllExport uint16_t& current();

            eProsima_user_DllExport void speed(float _v);
            eProsima_user_DllExport float speed() const;
            eProsima_user_DllExport float& speed();

            eProsima_user_DllExport void angle(float _v);
            eProsima_user_DllExport float angle() const;
            eProsima_user_DllExport float& angle();

            eProsima_user_DllExport void brake(float _v);
            eProsima_user_DllExport float brake() const;
            eProsima_user_DllExport float& brake();

            eProsima_user_DllExport void fault_code(uint16_t _v);
            eProsima_user_DllExport uint16_t fault_code() const;
            eProsima_user_DllExport uint16_t& fault_code();

            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static size_t getCdrSerializedSize(const vehicle_msgs::msg::VehicleState& data, size_t current_alignment = 0);
            eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr& cdr) const;
            eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr& cdr);
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static bool isKeyDefined();
            eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr& scdr) const;

        private:
            std_msgs::msg::Header m_header;
            uint8_t m_work_state;
            uint8_t m_work_mode;
            uint8_t m_control_model;
            uint8_t m_battery_capacity;
            uint16_t m_voltage;
            uint16_t m_current;
            float m_speed;
            float m_angle;
            float m_brake;
            uint16_t m_fault_code;
        };
    }
}

#endif // _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_VEHICLESTATE_H_
