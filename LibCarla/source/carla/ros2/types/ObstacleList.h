// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file ObstacleList.h
 */

#ifndef _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLELIST_H_
#define _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLELIST_H_

#include "Header.h"
#include "ObstacleItem.h"

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
#if defined(ObstacleList_SOURCE)
#define ObstacleList_DllAPI __declspec( dllexport )
#else
#define ObstacleList_DllAPI __declspec( dllimport )
#endif
#else
#define ObstacleList_DllAPI
#endif
#else
#define ObstacleList_DllAPI
#endif

namespace eprosima { namespace fastcdr { class Cdr; } }

namespace vehicle_msgs {
    namespace msg {
        class ObstacleList
        {
        public:
            eProsima_user_DllExport ObstacleList();
            eProsima_user_DllExport ~ObstacleList();
            eProsima_user_DllExport ObstacleList(const ObstacleList& x);
            eProsima_user_DllExport ObstacleList(ObstacleList&& x) noexcept;
            eProsima_user_DllExport ObstacleList& operator =(const ObstacleList& x);
            eProsima_user_DllExport ObstacleList& operator =(ObstacleList&& x) noexcept;
            eProsima_user_DllExport bool operator ==(const ObstacleList& x) const;
            eProsima_user_DllExport bool operator !=(const ObstacleList& x) const;

            eProsima_user_DllExport void header(const std_msgs::msg::Header& _header);
            eProsima_user_DllExport void header(std_msgs::msg::Header&& _header);
            eProsima_user_DllExport const std_msgs::msg::Header& header() const;
            eProsima_user_DllExport std_msgs::msg::Header& header();

            eProsima_user_DllExport void obstacles(const std::vector<vehicle_msgs::msg::ObstacleItem>& _obstacles);
            eProsima_user_DllExport void obstacles(std::vector<vehicle_msgs::msg::ObstacleItem>&& _obstacles);
            eProsima_user_DllExport const std::vector<vehicle_msgs::msg::ObstacleItem>& obstacles() const;
            eProsima_user_DllExport std::vector<vehicle_msgs::msg::ObstacleItem>& obstacles();

            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static size_t getCdrSerializedSize(const vehicle_msgs::msg::ObstacleList& data, size_t current_alignment = 0);
            eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr& cdr) const;
            eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr& cdr);
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static bool isKeyDefined();
            eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr& scdr) const;

        private:
            std_msgs::msg::Header m_header;
            std::vector<vehicle_msgs::msg::ObstacleItem> m_obstacles;
        };
    }
}

#endif // _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLELIST_H_
