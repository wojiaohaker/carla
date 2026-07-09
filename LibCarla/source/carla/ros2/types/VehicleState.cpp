// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file VehicleState.cpp
 * This source file contains the definition of the described types in the IDL file.
 */

#ifdef _WIN32
namespace { char dummy; }
#endif

#include "VehicleState.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;
#include <utility>

#define vehicle_msgs_msg_VehicleState_max_cdr_typesize 300ULL;
#define vehicle_msgs_msg_VehicleState_max_key_cdr_typesize 0ULL;

vehicle_msgs::msg::VehicleState::VehicleState()
{
    m_work_state = 0;
    m_work_mode = 0;
    m_control_model = 0;
    m_battery_capacity = 100;
    m_voltage = 0;
    m_current = 0;
    m_speed = 0.0f;
    m_angle = 0.0f;
    m_brake = 0.0f;
    m_fault_code = 0;
}

vehicle_msgs::msg::VehicleState::~VehicleState() {}

vehicle_msgs::msg::VehicleState::VehicleState(const VehicleState& x)
{
    m_header = x.m_header;
    m_work_state = x.m_work_state;
    m_work_mode = x.m_work_mode;
    m_control_model = x.m_control_model;
    m_battery_capacity = x.m_battery_capacity;
    m_voltage = x.m_voltage;
    m_current = x.m_current;
    m_speed = x.m_speed;
    m_angle = x.m_angle;
    m_brake = x.m_brake;
    m_fault_code = x.m_fault_code;
}

vehicle_msgs::msg::VehicleState::VehicleState(VehicleState&& x) noexcept
{
    m_header = std::move(x.m_header);
    m_work_state = x.m_work_state;
    m_work_mode = x.m_work_mode;
    m_control_model = x.m_control_model;
    m_battery_capacity = x.m_battery_capacity;
    m_voltage = x.m_voltage;
    m_current = x.m_current;
    m_speed = x.m_speed;
    m_angle = x.m_angle;
    m_brake = x.m_brake;
    m_fault_code = x.m_fault_code;
}

vehicle_msgs::msg::VehicleState& vehicle_msgs::msg::VehicleState::operator =(const VehicleState& x)
{
    m_header = x.m_header;
    m_work_state = x.m_work_state;
    m_work_mode = x.m_work_mode;
    m_control_model = x.m_control_model;
    m_battery_capacity = x.m_battery_capacity;
    m_voltage = x.m_voltage;
    m_current = x.m_current;
    m_speed = x.m_speed;
    m_angle = x.m_angle;
    m_brake = x.m_brake;
    m_fault_code = x.m_fault_code;
    return *this;
}

vehicle_msgs::msg::VehicleState& vehicle_msgs::msg::VehicleState::operator =(VehicleState&& x) noexcept
{
    m_header = std::move(x.m_header);
    m_work_state = x.m_work_state;
    m_work_mode = x.m_work_mode;
    m_control_model = x.m_control_model;
    m_battery_capacity = x.m_battery_capacity;
    m_voltage = x.m_voltage;
    m_current = x.m_current;
    m_speed = x.m_speed;
    m_angle = x.m_angle;
    m_brake = x.m_brake;
    m_fault_code = x.m_fault_code;
    return *this;
}

bool vehicle_msgs::msg::VehicleState::operator ==(const VehicleState& x) const
{
    return (m_header == x.m_header &&
            m_work_state == x.m_work_state &&
            m_work_mode == x.m_work_mode &&
            m_control_model == x.m_control_model &&
            m_battery_capacity == x.m_battery_capacity &&
            m_voltage == x.m_voltage &&
            m_current == x.m_current &&
            m_speed == x.m_speed &&
            m_angle == x.m_angle &&
            m_brake == x.m_brake &&
            m_fault_code == x.m_fault_code);
}

bool vehicle_msgs::msg::VehicleState::operator !=(const VehicleState& x) const { return !(*this == x); }

size_t vehicle_msgs::msg::VehicleState::getMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return vehicle_msgs_msg_VehicleState_max_cdr_typesize;
}

size_t vehicle_msgs::msg::VehicleState::getCdrSerializedSize(const vehicle_msgs::msg::VehicleState& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;
    current_alignment += std_msgs::msg::Header::getCdrSerializedSize(data.header(), current_alignment);
    // work_state, work_mode, control_model, battery_capacity (uint8 x4)
    current_alignment += 4;
    // voltage, current (uint16 x2)
    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);
    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);
    // speed, angle, brake (float x3)
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    // fault_code (uint16)
    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);
    return current_alignment - initial_alignment;
}

void vehicle_msgs::msg::VehicleState::serialize(eprosima::fastcdr::Cdr& scdr) const
{
    scdr << m_header;
    scdr << m_work_state;
    scdr << m_work_mode;
    scdr << m_control_model;
    scdr << m_battery_capacity;
    scdr << m_voltage;
    scdr << m_current;
    scdr << m_speed;
    scdr << m_angle;
    scdr << m_brake;
    scdr << m_fault_code;
}

void vehicle_msgs::msg::VehicleState::deserialize(eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> m_header;
    dcdr >> m_work_state;
    dcdr >> m_work_mode;
    dcdr >> m_control_model;
    dcdr >> m_battery_capacity;
    dcdr >> m_voltage;
    dcdr >> m_current;
    dcdr >> m_speed;
    dcdr >> m_angle;
    dcdr >> m_brake;
    dcdr >> m_fault_code;
}

void vehicle_msgs::msg::VehicleState::header(const std_msgs::msg::Header& _v) { m_header = _v; }
void vehicle_msgs::msg::VehicleState::header(std_msgs::msg::Header&& _v) { m_header = std::move(_v); }
const std_msgs::msg::Header& vehicle_msgs::msg::VehicleState::header() const { return m_header; }
std_msgs::msg::Header& vehicle_msgs::msg::VehicleState::header() { return m_header; }

void vehicle_msgs::msg::VehicleState::work_state(uint8_t _v) { m_work_state = _v; }
uint8_t vehicle_msgs::msg::VehicleState::work_state() const { return m_work_state; }
uint8_t& vehicle_msgs::msg::VehicleState::work_state() { return m_work_state; }

void vehicle_msgs::msg::VehicleState::work_mode(uint8_t _v) { m_work_mode = _v; }
uint8_t vehicle_msgs::msg::VehicleState::work_mode() const { return m_work_mode; }
uint8_t& vehicle_msgs::msg::VehicleState::work_mode() { return m_work_mode; }

void vehicle_msgs::msg::VehicleState::control_model(uint8_t _v) { m_control_model = _v; }
uint8_t vehicle_msgs::msg::VehicleState::control_model() const { return m_control_model; }
uint8_t& vehicle_msgs::msg::VehicleState::control_model() { return m_control_model; }

void vehicle_msgs::msg::VehicleState::battery_capacity(uint8_t _v) { m_battery_capacity = _v; }
uint8_t vehicle_msgs::msg::VehicleState::battery_capacity() const { return m_battery_capacity; }
uint8_t& vehicle_msgs::msg::VehicleState::battery_capacity() { return m_battery_capacity; }

void vehicle_msgs::msg::VehicleState::voltage(uint16_t _v) { m_voltage = _v; }
uint16_t vehicle_msgs::msg::VehicleState::voltage() const { return m_voltage; }
uint16_t& vehicle_msgs::msg::VehicleState::voltage() { return m_voltage; }

void vehicle_msgs::msg::VehicleState::current(uint16_t _v) { m_current = _v; }
uint16_t vehicle_msgs::msg::VehicleState::current() const { return m_current; }
uint16_t& vehicle_msgs::msg::VehicleState::current() { return m_current; }

void vehicle_msgs::msg::VehicleState::speed(float _v) { m_speed = _v; }
float vehicle_msgs::msg::VehicleState::speed() const { return m_speed; }
float& vehicle_msgs::msg::VehicleState::speed() { return m_speed; }

void vehicle_msgs::msg::VehicleState::angle(float _v) { m_angle = _v; }
float vehicle_msgs::msg::VehicleState::angle() const { return m_angle; }
float& vehicle_msgs::msg::VehicleState::angle() { return m_angle; }

void vehicle_msgs::msg::VehicleState::brake(float _v) { m_brake = _v; }
float vehicle_msgs::msg::VehicleState::brake() const { return m_brake; }
float& vehicle_msgs::msg::VehicleState::brake() { return m_brake; }

void vehicle_msgs::msg::VehicleState::fault_code(uint16_t _v) { m_fault_code = _v; }
uint16_t vehicle_msgs::msg::VehicleState::fault_code() const { return m_fault_code; }
uint16_t& vehicle_msgs::msg::VehicleState::fault_code() { return m_fault_code; }

size_t vehicle_msgs::msg::VehicleState::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return vehicle_msgs_msg_VehicleState_max_key_cdr_typesize;
}

bool vehicle_msgs::msg::VehicleState::isKeyDefined() { return false; }

void vehicle_msgs::msg::VehicleState::serializeKey(eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
}
