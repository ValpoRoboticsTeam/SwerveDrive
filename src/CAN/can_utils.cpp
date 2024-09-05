#include "CAN/can_utils.hpp"

// returns 32 bit can identifier based on FRC protocol for interacting
// with motor controller
uint32_t genCanFrameID(can_id_params* msg_params) {
    uint32_t id = 4;  // 1 0 0  - configuration bits determined from documentation. See line 39 in kernel documentation url provided

    id <<= DEVICE_BITS;
    id |= msg_params->deviceType;

    id <<= MANUFACTURER_BITS;
    id |= msg_params->manufacturer;

    id <<= API_CLASS_BITS;
    id |= msg_params->apiClass;

    id <<= API_INDEX_BITS;
    id |= msg_params->apiIndex;

    id <<= DEVICE_NUMBER_BITS;
    id |= msg_params->deviceNumber;

    return id;
}


// Decodes an integer into its components based on the 
// FRC Can frame encoding using bit shifting and masking
void decodeCanFrameID(uint32_t msg, can_id_params* msg_params) {
    msg_params->deviceNumber = msg & (0xFF >> (8 - DEVICE_NUMBER_BITS));
    msg >>= DEVICE_NUMBER_BITS;

    msg_params->apiIndex = msg & (0xFF >> (8 - API_INDEX_BITS));
    msg >>= API_INDEX_BITS;

    msg_params->apiClass = msg & (0xFF >> (8 - API_CLASS_BITS));
    msg >>= API_CLASS_BITS;

    msg_params->manufacturer = msg & (0xFF >> (8 - MANUFACTURER_BITS));
    msg >>= MANUFACTURER_BITS;

    msg_params->deviceType = msg & (0xFF >> (8 - DEVICE_BITS));
    msg >>= DEVICE_BITS;
}


// bitmasks to get the device
int decodeCanFrameDevice(uint32_t msg) {
    return msg & (0xFF >> (8 - DEVICE_NUMBER_BITS));
}


// populates an array of bytes from an unsigned integer with the msb at index numBytes - 1
// and the lsb at index 0
void uint64ToBytes(uint64_t n, uint8_t bytes[], uint8_t numBytes) {
    for(int i = 0; i < numBytes; i++) {
        uint8_t byte = n & 0xFF;
        bytes[i] = byte;
        n >>= 8;
    }
}


// populates an array of bytes from a signed integer with the msb at index numBytes - 1
// and the lsb at index 0
void int64ToBytes(int64_t n, uint8_t bytes[], uint8_t numBytes) {
    uint64_t intN = *(uint64_t*)&n;    // convert from signed to unsigned but keep bits the same by casting.
                                       // This is necessary to use bit shifting operations to convert this to 
                                       // bytes
    uint64ToBytes(intN, bytes, numBytes);
}

// populates an array of bytes from a floating point number
// with the msb at index numBytes - 1 and the lsb at index 0
void floatToBytes(float n, uint8_t bytes[], uint8_t numBytes) {
    uint64_t intN = *(uint64_t*)&n;    // convert the float to an integer but keeping bits the same by casting
                                       // this is necessary to use bit shifting operations to convert this to 
                                       // bytes
    uint64ToBytes(intN, bytes, numBytes);
}


// given an array of bytes with the msb at index numBytes - 1 and lsb and index
// 0, creates and returns a 64 bit integer
uint64_t bytesToUnsignedInt64(uint8_t bytes[], uint8_t numBytes) {
    uint64_t intN = 0;
    for(int i = 0; i < numBytes; i++) {
        intN |= bytes[i] << (8 * i);
    }
    intN |= bytes[numBytes];

    return intN;  // cast int memory to reinterpret data as a float
}


// given an array of bytes with the msb at index numBytes - 1 and lsb and index
// 0, creates and returns an unsigned 64 bit integer
int64_t bytesToSignedInt64(uint8_t bytes[], uint8_t numBytes) {
    uint64_t intN = bytesToUnsignedInt64(bytes, numBytes);

    return *(int64_t*)&intN;  // cast int memory to reinterpret data as a float
}


// given an array of bytes with the msb at index 0 and lsb and index
// numBytes - 1, creates and returns floating point number
float bytesToFloat(uint8_t bytes[], uint8_t numBytes) {
    uint64_t intN = bytesToUnsignedInt64(bytes, numBytes);

    return *(float*)&intN;  // cast int memory to reinterpret data as a float
}
