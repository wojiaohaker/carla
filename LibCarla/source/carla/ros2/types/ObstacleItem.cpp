// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file ObstacleItem.cpp
 */

#ifdef _WIN32
namespace { char dummy; }
#endif

#include "ObstacleItem.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;
#include <utility>

#define vehicle_msgs_msg_ObstacleItem_max_cdr_typesize 80ULL;
#define vehicle_msgs_msg_ObstacleItem_max_key_cdr_typesize 0ULL;

vehicle_msgs::msg::ObstacleItem::ObstacleItem()
{
    m_id = 0;
    m_type = 0;
    m_x = 0.0;
    m_y = 0.0;
    m_length = 0.0;
    m_width = 0.0;
    m_height = 0.0;
    m_course = 0.0;
    m_speed = 0.0;
}

vehicle_msgs::msg::ObstacleItem::~ObstacleItem() {}

vehicle_msgs::msg::ObstacleItem::ObstacleItem(const ObstacleItem& x)
{
    m_id = x.m_id; m_type = x.m_type;
    m_x = x.m_x; m_y = x.m_y;
    m_length = x.m_length; m_width = x.m_width; m_height = x.m_height;
    m_course = x.m_course; m_speed = x.m_speed;
}

vehicle_msgs::msg::ObstacleItem::ObstacleItem(ObstacleItem&& x) noexcept
{
    m_id = x.m_id; m_type = x.m_type;
    m_x = x.m_x; m_y = x.m_y;
    m_length = x.m_length; m_width = x.m_width; m_height = x.m_height;
    m_course = x.m_course; m_speed = x.m_speed;
}

vehicle_msgs::msg::ObstacleItem& vehicle_msgs::msg::ObstacleItem::operator =(const ObstacleItem& x)
{
    m_id = x.m_id; m_type = x.m_type;
    m_x = x.m_x; m_y = x.m_y;
    m_length = x.m_length; m_width = x.m_width; m_height = x.m_height;
    m_course = x.m_course; m_speed = x.m_speed;
    return *this;
}

vehicle_msgs::msg::ObstacleItem& vehicle_msgs::msg::ObstacleItem::operator =(ObstacleItem&& x) noexcept
{
    m_id = x.m_id; m_type = x.m_type;
    m_x = x.m_x; m_y = x.m_y;
    m_length = x.m_length; m_width = x.m_width; m_height = x.m_height;
    m_course = x.m_course; m_speed = x.m_speed;
    return *this;
}

bool vehicle_msgs::msg::ObstacleItem::operator ==(const ObstacleItem& x) const
{
    return (m_id == x.m_id && m_type == x.m_type &&
            m_x == x.m_x && m_y == x.m_y &&
            m_length == x.m_length && m_width == x.m_width && m_height == x.m_height &&
            m_course == x.m_course && m_speed == x.m_speed);
}

bool vehicle_msgs::msg::ObstacleItem::operator !=(const ObstacleItem& x) const { return !(*this == x); }

size_t vehicle_msgs::msg::ObstacleItem::getMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return vehicle_msgs_msg_ObstacleItem_max_cdr_typesize;
}

size_t vehicle_msgs::msg::ObstacleItem::getCdrSerializedSize(const vehicle_msgs::msg::ObstacleItem& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;
    // id (int32)
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    // type (uint8)
    current_alignment += 1;
    // x, y, length, width, height, course, speed (double x7)
    current_alignment += 7 * (8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8));
    return current_alignment - initial_alignment;
}

void vehicle_msgs::msg::ObstacleItem::serialize(eprosima::fastcdr::Cdr& scdr) const
{
    scdr << m_id << m_type << m_x << m_y << m_length << m_width << m_height << m_course << m_speed;
}

void vehicle_msgs::msg::ObstacleItem::deserialize(eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> m_id >> m_type >> m_x >> m_y >> m_length >> m_width >> m_height >> m_course >> m_speed;
}

void vehicle_msgs::msg::ObstacleItem::id(int32_t _v) { m_id = _v; }
int32_t vehicle_msgs::msg::ObstacleItem::id() const { return m_id; }
int32_t& vehicle_msgs::msg::ObstacleItem::id() { return m_id; }

void vehicle_msgs::msg::ObstacleItem::type(uint8_t _v) { m_type = _v; }
uint8_t vehicle_msgs::msg::ObstacleItem::type() const { return m_type; }
uint8_t& vehicle_msgs::msg::ObstacleItem::type() { return m_type; }

void vehicle_msgs::msg::ObstacleItem::x(double _v) { m_x = _v; }
double vehicle_msgs::msg::ObstacleItem::x() const { return m_x; }
double& vehicle_msgs::msg::ObstacleItem::x() { return m_x; }

void vehicle_msgs::msg::ObstacleItem::y(double _v) { m_y = _v; }
double vehicle_msgs::msg::ObstacleItem::y() const { return m_y; }
double& vehicle_msgs::msg::ObstacleItem::y() { return m_y; }

void vehicle_msgs::msg::ObstacleItem::length(double _v) { m_length = _v; }
double vehicle_msgs::msg::ObstacleItem::length() const { return m_length; }
double& vehicle_msgs::msg::ObstacleItem::length() { return m_length; }

void vehicle_msgs::msg::ObstacleItem::width(double _v) { m_width = _v; }
double vehicle_msgs::msg::ObstacleItem::width() const { return m_width; }
double& vehicle_msgs::msg::ObstacleItem::width() { return m_width; }

void vehicle_msgs::msg::ObstacleItem::height(double _v) { m_height = _v; }
double vehicle_msgs::msg::ObstacleItem::height() const { return m_height; }
double& vehicle_msgs::msg::ObstacleItem::height() { return m_height; }

void vehicle_msgs::msg::ObstacleItem::course(double _v) { m_course = _v; }
double vehicle_msgs::msg::ObstacleItem::course() const { return m_course; }
double& vehicle_msgs::msg::ObstacleItem::course() { return m_course; }

void vehicle_msgs::msg::ObstacleItem::speed(double _v) { m_speed = _v; }
double vehicle_msgs::msg::ObstacleItem::speed() const { return m_speed; }
double& vehicle_msgs::msg::ObstacleItem::speed() { return m_speed; }

size_t vehicle_msgs::msg::ObstacleItem::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return vehicle_msgs_msg_ObstacleItem_max_key_cdr_typesize;
}

bool vehicle_msgs::msg::ObstacleItem::isKeyDefined() { return false; }

void vehicle_msgs::msg::ObstacleItem::serializeKey(eprosima::fastcdr::Cdr& scdr) const { (void) scdr; }
