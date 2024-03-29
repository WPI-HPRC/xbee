//
// Created by William Scheirey on 3/18/24.
//

#ifndef HPRC_CIRCULARBUFFER_HPP
#define HPRC_CIRCULARBUFFER_HPP

#include <cstdlib>
#include <cstring>

typedef struct
{
    unsigned int length;
    unsigned int dataSize_bytes;
    uint8_t *dataPtr;
    uint8_t *data;
} CircularBuffer;


inline uint8_t *circularBufferGetValueAtIndex(CircularBuffer *buffer, int index)
{
    return &(buffer->data[((buffer->dataPtr - buffer->data) + index * buffer->dataSize_bytes) %
                          (buffer->length * buffer->dataSize_bytes)]);
}

inline uint8_t *circularBufferGetLastValue(CircularBuffer *buffer)
{
    return circularBufferGetValueAtIndex(buffer, -1);
}

inline CircularBuffer *circularBufferCreate(unsigned int length, unsigned int dataSize_bytes)
{
    auto *buffer = (CircularBuffer *) malloc(dataSize_bytes * length);

    buffer->length = length;
    buffer->dataSize_bytes = dataSize_bytes;

    buffer->data = (uint8_t *) malloc(dataSize_bytes * length);

    buffer->dataPtr = &buffer->data[0];

    return buffer;
}

inline void circularBufferAdd(CircularBuffer *buffer, uint8_t *data, size_t length_bytes)
{
    memcpy(buffer->dataPtr, data, length_bytes);

    if (buffer->dataPtr == &buffer->data[buffer->length - 1])
    {
        buffer->dataPtr = &buffer->data[0];
    }
    else
    {
        buffer->dataPtr += buffer->dataSize_bytes;
    }
}

#endif //HPRC_CIRCULARBUFFER_HPP
