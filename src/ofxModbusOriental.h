#ifndef OFXMODBUSORIENTAL_H
#define OFXMODBUSORIENTAL_H

#define OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN namespace ofxModbusOriental {
#define OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END }

#include <stdint.h>
#include <array>

// hash expansion
namespace std
{
    template <typename T>
    struct hash
    {
        static_assert(is_enum<T>::value, "this hash only works for enumeration types");
        size_t operator() (T x) const noexcept
        {
            using type = typename underlying_type<T>::type;
            return hash<type>{}(static_cast<type>(x));
        }
    };
}

struct EnumClassHash
{
    template <typename T>
    std::size_t operator() (T t) const noexcept
    {
        return static_cast<std::size_t>(t);
    }
};

OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_BEGIN
OFX_MODBUS_ORIENTAL_MOTOR_NAMESPACE_END
namespace ofxOriental = ofxModbusOriental;

#include "ofxModbusOrientalController.h"

#endif /* OFXMODBUSORIENTAL_H */
