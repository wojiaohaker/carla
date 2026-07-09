// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Licensed under the Apache License, Version 2.0.

/*!
 * @file VehicleStatePubSubTypes.cpp
 */

#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "VehicleStatePubSubTypes.h"

using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;
using InstanceHandle_t = eprosima::fastrtps::rtps::InstanceHandle_t;

namespace vehicle_msgs {
    namespace msg {
        VehicleStatePubSubType::VehicleStatePubSubType()
        {
            setName("vehicle_msgs::msg::dds_::VehicleState_");
            auto type_size = VehicleState::getMaxCdrSerializedSize();
            type_size += eprosima::fastcdr::Cdr::alignment(type_size, 4);
            m_typeSize = static_cast<uint32_t>(type_size) + 4;
            m_isGetKeyDefined = VehicleState::isKeyDefined();
            size_t keyLength = VehicleState::getKeyMaxCdrSerializedSize() > 16 ?
                    VehicleState::getKeyMaxCdrSerializedSize() : 16;
            m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
            memset(m_keyBuffer, 0, keyLength);
        }

        VehicleStatePubSubType::~VehicleStatePubSubType()
        {
            if (m_keyBuffer != nullptr) free(m_keyBuffer);
        }

        bool VehicleStatePubSubType::serialize(void* data, SerializedPayload_t* payload)
        {
            VehicleState* p_type = static_cast<VehicleState*>(data);
            eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size);
            eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
            payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
            ser.serialize_encapsulation();
            try { p_type->serialize(ser); }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException&) { return false; }
            payload->length = static_cast<uint32_t>(ser.getSerializedDataLength());
            return true;
        }

        bool VehicleStatePubSubType::deserialize(SerializedPayload_t* payload, void* data)
        {
            try
            {
                VehicleState* p_type = static_cast<VehicleState*>(data);
                eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length);
                eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
                deser.read_encapsulation();
                payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
                p_type->deserialize(deser);
            }
            catch (eprosima::fastcdr::exception::NotEnoughMemoryException&) { return false; }
            return true;
        }

        std::function<uint32_t()> VehicleStatePubSubType::getSerializedSizeProvider(void* data)
        {
            return [data]() -> uint32_t {
                return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<VehicleState*>(data))) + 4u;
            };
        }

        void* VehicleStatePubSubType::createData() { return reinterpret_cast<void*>(new VehicleState()); }
        void VehicleStatePubSubType::deleteData(void* data) { delete(reinterpret_cast<VehicleState*>(data)); }

        bool VehicleStatePubSubType::getKey(void* data, InstanceHandle_t* handle, bool force_md5)
        {
            if (!m_isGetKeyDefined) return false;
            VehicleState* p_type = static_cast<VehicleState*>(data);
            eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer), VehicleState::getKeyMaxCdrSerializedSize());
            eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
            p_type->serializeKey(ser);
            if (force_md5 || VehicleState::getKeyMaxCdrSerializedSize() > 16)
            {
                m_md5.init();
                m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
                m_md5.finalize();
                for (uint8_t i = 0; i < 16; ++i) handle->value[i] = m_md5.digest[i];
            }
            else
            {
                for (uint8_t i = 0; i < 16; ++i) handle->value[i] = m_keyBuffer[i];
            }
            return true;
        }
    }
}
