// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file ObstacleListPubSubTypes.cpp
 */

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>
#include "ObstacleListPubSubTypes.h"

using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;
using InstanceHandle_t = eprosima::fastrtps::rtps::InstanceHandle_t;

namespace vehicle_msgs {
    namespace msg {
        ObstacleListPubSubType::ObstacleListPubSubType()
        {
            setName("vehicle_msgs::msg::dds_::ObstacleList_");
            auto type_size = ObstacleList::getMaxCdrSerializedSize();
            type_size += eprosima::fastcdr::Cdr::alignment(type_size, 4);
            m_typeSize = static_cast<uint32_t>(type_size) + 4;
            m_isGetKeyDefined = ObstacleList::isKeyDefined();
            size_t keyLength = ObstacleList::getKeyMaxCdrSerializedSize() > 16 ?
                    ObstacleList::getKeyMaxCdrSerializedSize() : 16;
            m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
            memset(m_keyBuffer, 0, keyLength);
        }

        ObstacleListPubSubType::~ObstacleListPubSubType()
        {
            if (m_keyBuffer != nullptr) free(m_keyBuffer);
        }

        bool ObstacleListPubSubType::serialize(void* data, SerializedPayload_t* payload)
        {
            ObstacleList* p_type = static_cast<ObstacleList*>(data);
            eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size);
            eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
            payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
            ser.serialize_encapsulation();
            try { p_type->serialize(ser); }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException&) { return false; }
            payload->length = static_cast<uint32_t>(ser.getSerializedDataLength());
            return true;
        }

        bool ObstacleListPubSubType::deserialize(SerializedPayload_t* payload, void* data)
        {
            try
            {
                ObstacleList* p_type = static_cast<ObstacleList*>(data);
                eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length);
                eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
                deser.read_encapsulation();
                payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
                p_type->deserialize(deser);
            }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException&) { return false; }
            return true;
        }

        std::function<uint32_t()> ObstacleListPubSubType::getSerializedSizeProvider(void* data)
        {
            return [data]() -> uint32_t {
                return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<ObstacleList*>(data))) + 4u;
            };
        }

        void* ObstacleListPubSubType::createData() { return reinterpret_cast<void*>(new ObstacleList()); }
        void ObstacleListPubSubType::deleteData(void* data) { delete(reinterpret_cast<ObstacleList*>(data)); }

        bool ObstacleListPubSubType::getKey(void* data, InstanceHandle_t* handle, bool force_md5)
        {
            if (!m_isGetKeyDefined) return false;
            ObstacleList* p_type = static_cast<ObstacleList*>(data);
            eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer), ObstacleList::getKeyMaxCdrSerializedSize());
            eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
            p_type->serializeKey(ser);
            if (force_md5 || ObstacleList::getKeyMaxCdrSerializedSize() > 16)
            {
                m_md5.init();
                m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
                m_md5.finalize();
                for (uint8_t i = 0; i < 16; ++i) handle->value[i] = m_md5.digest[i];
            }
            else { for (uint8_t i = 0; i < 16; ++i) handle->value[i] = m_keyBuffer[i]; }
            return true;
        }
    }
}
