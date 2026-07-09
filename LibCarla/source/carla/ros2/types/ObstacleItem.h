// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file ObstacleItem.h
 */

#ifndef _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLEITEM_H_
#define _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLEITEM_H_

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
#if defined(ObstacleItem_SOURCE)
#define ObstacleItem_DllAPI __declspec( dllexport )
#else
#define ObstacleItem_DllAPI __declspec( dllimport )
#endif
#else
#define ObstacleItem_DllAPI
#endif
#else
#define ObstacleItem_DllAPI
#endif

namespace eprosima { namespace fastcdr { class Cdr; } }

namespace vehicle_msgs {
    namespace msg {
        class ObstacleItem
        {
        public:
            eProsima_user_DllExport ObstacleItem();
            eProsima_user_DllExport ~ObstacleItem();
            eProsima_user_DllExport ObstacleItem(const ObstacleItem& x);
            eProsima_user_DllExport ObstacleItem(ObstacleItem&& x) noexcept;
            eProsima_user_DllExport ObstacleItem& operator =(const ObstacleItem& x);
            eProsima_user_DllExport ObstacleItem& operator =(ObstacleItem&& x) noexcept;
            eProsima_user_DllExport bool operator ==(const ObstacleItem& x) const;
            eProsima_user_DllExport bool operator !=(const ObstacleItem& x) const;

            eProsima_user_DllExport void id(int32_t _v);
            eProsima_user_DllExport int32_t id() const;
            eProsima_user_DllExport int32_t& id();

            eProsima_user_DllExport void type(uint8_t _v);
            eProsima_user_DllExport uint8_t type() const;
            eProsima_user_DllExport uint8_t& type();

            eProsima_user_DllExport void x(double _v);
            eProsima_user_DllExport double x() const;
            eProsima_user_DllExport double& x();

            eProsima_user_DllExport void y(double _v);
            eProsima_user_DllExport double y() const;
            eProsima_user_DllExport double& y();

            eProsima_user_DllExport void length(double _v);
            eProsima_user_DllExport double length() const;
            eProsima_user_DllExport double& length();

            eProsima_user_DllExport void width(double _v);
            eProsima_user_DllExport double width() const;
            eProsima_user_DllExport double& width();

            eProsima_user_DllExport void height(double _v);
            eProsima_user_DllExport double height() const;
            eProsima_user_DllExport double& height();

            eProsima_user_DllExport void course(double _v);
            eProsima_user_DllExport double course() const;
            eProsima_user_DllExport double& course();

            eProsima_user_DllExport void speed(double _v);
            eProsima_user_DllExport double speed() const;
            eProsima_user_DllExport double& speed();

            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static size_t getCdrSerializedSize(const vehicle_msgs::msg::ObstacleItem& data, size_t current_alignment = 0);
            eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr& cdr) const;
            eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr& cdr);
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static bool isKeyDefined();
            eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr& scdr) const;

        private:
            int32_t m_id;
            uint8_t m_type;
            double m_x;
            double m_y;
            double m_length;
            double m_width;
            double m_height;
            double m_course;
            double m_speed;
        };
    }
}

#endif // _FAST_DDS_GENERATED_VEHICLE_MSGS_MSG_OBSTACLEITEM_H_
