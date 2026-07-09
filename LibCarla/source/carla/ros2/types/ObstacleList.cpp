// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file ObstacleList.cpp
 */

#ifdef _WIN32
namespace { char dummy; }
#endif

#include "ObstacleList.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;
#include <utility>

vehicle_msgs::msg::ObstacleList::ObstacleList() {}
vehicle_msgs::msg::ObstacleList::~ObstacleList() {}

vehicle_msgs::msg::ObstacleList::ObstacleList(const ObstacleList& x)
{
    m_header = x.m_header;
    m_obstacles = x.m_obstacles;
}

vehicle_msgs::msg::ObstacleList::ObstacleList(ObstacleList&& x) noexcept
{
    m_header = std::move(x.m_header);
    m_obstacles = std::move(x.m_obstacles);
}

vehicle_msgs::msg::ObstacleList& vehicle_msgs::msg::ObstacleList::operator =(const ObstacleList& x)
{
    m_header = x.m_header;
    m_obstacles = x.m_obstacles;
    return *this;
}

vehicle_msgs::msg::ObstacleList& vehicle_msgs::msg::ObstacleList::operator =(ObstacleList&& x) noexcept
{
    m_header = std::move(x.m_header);
    m_obstacles = std::move(x.m_obstacles);
    return *this;
}

bool vehicle_msgs::msg::ObstacleList::operator ==(const ObstacleList& x) const
{
    return (m_header == x.m_header && m_obstacles == x.m_obstacles);
}

bool vehicle_msgs::msg::ObstacleList::operator !=(const ObstacleList& x) const { return !(*this == x); }

size_t vehicle_msgs::msg::ObstacleList::getMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    // Dynamic size - no fixed max. Return a reasonable estimate.
    return 2048;
}

size_t vehicle_msgs::msg::ObstacleList::getCdrSerializedSize(const vehicle_msgs::msg::ObstacleList& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
    current_alignment += std_msgs::msg::Header::getCdrSerializedSize(data.header(), current_alignment);
    // obstacles vector: 4 bytes for length + each element
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    for (const auto& item : data.obstacles())
    {
        current_alignment += ObstacleItem::getCdrSerializedSize(item, current_alignment);
    }
    return current_alignment - initial_alignment;
}

void vehicle_msgs::msg::ObstacleList::serialize(eprosima::fastcdr::Cdr& scdr) const
{
    scdr << m_header;
    scdr << static_cast<uint32_t>(m_obstacles.size());
    for (const auto& item : m_obstacles)
    {
        scdr << item;
    }
}

void vehicle_msgs::msg::ObstacleList::deserialize(eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> m_header;
    uint32_t length;
    dcdr >> length;
    m_obstacles.resize(length);
    for (auto& item : m_obstacles)
    {
        dcdr >> item;
    }
}

void vehicle_msgs::msg::ObstacleList::header(const std_msgs::msg::Header& _v) { m_header = _v; }
void vehicle_msgs::msg::ObstacleList::header(std_msgs::msg::Header&& _v) { m_header = std::move(_v); }
const std_msgs::msg::Header& vehicle_msgs::msg::ObstacleList::header() const { return m_header; }
std_msgs::msg::Header& vehicle_msgs::msg::ObstacleList::header() { return m_header; }

void vehicle_msgs::msg::ObstacleList::obstacles(const std::vector<vehicle_msgs::msg::ObstacleItem>& _v) { m_obstacles = _v; }
void vehicle_msgs::msg::ObstacleList::obstacles(std::vector<vehicle_msgs::msg::ObstacleItem>&& _v) { m_obstacles = std::move(_v); }
const std::vector<vehicle_msgs::msg::ObstacleItem>& vehicle_msgs::msg::ObstacleList::obstacles() const { return m_obstacles; }
std::vector<vehicle_msgs::msg::ObstacleItem>& vehicle_msgs::msg::ObstacleList::obstacles() { return m_obstacles; }

size_t vehicle_msgs::msg::ObstacleList::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return 0;
}

bool vehicle_msgs::msg::ObstacleList::isKeyDefined() { return false; }
void vehicle_msgs::msg::ObstacleList::serializeKey(eprosima::fastcdr::Cdr& scdr) const { (void) scdr; }
