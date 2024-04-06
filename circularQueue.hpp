//
// Created by William Scheirey on 3/29/24.
//

#ifndef HPRC_CIRCULARQUEUE_HPP
#define HPRC_CIRCULARQUEUE_HPP

#include <iostream>

template<typename T>

struct CircularQueue
{
    unsigned int length;
    unsigned int dataSize_bytes;
    T *dataPtr;
    T *readPtr;
    T *data;
};

template<typename T>
inline CircularQueue<T> *circularQueueCreate(unsigned int length)
{
    auto *buffer = (CircularQueue<T> *) malloc(sizeof(CircularQueue<T>));

    buffer->length = length;
    buffer->dataSize_bytes = sizeof(T);

    buffer->data = (T *) malloc(length * buffer->dataSize_bytes);

    buffer->dataPtr = buffer->data;
    buffer->readPtr = buffer->dataPtr;

    return buffer;
}

template<typename T>
inline void
circularQueuePeek(CircularQueue<T> *queue, T *outBuffer, unsigned int length, unsigned int *bytes_overflowed = nullptr)
{
    unsigned int length_bytes = length * queue->dataSize_bytes;

    if (queue->data + queue->length * queue->dataSize_bytes - queue->readPtr >= length_bytes)
    {
        // No wrap-around, data is contiguous
        memcpy(outBuffer, queue->readPtr, length_bytes);
        if (bytes_overflowed != nullptr)
        {
            *bytes_overflowed = 0;
        }
    }
    else
    {
        // Wrap-around, data spans the end of the buffer
        unsigned int bytes_until_wrap = (queue->data + queue->length * queue->dataSize_bytes) - queue->readPtr;
        unsigned int bytes_after_wrap = length_bytes - bytes_until_wrap;

        memcpy(outBuffer, queue->readPtr, bytes_until_wrap);
        memcpy(outBuffer + bytes_until_wrap, queue->data, bytes_after_wrap);

        if (bytes_overflowed != nullptr)
        {
            *bytes_overflowed = bytes_after_wrap;
        }
    }
}

template<typename T>
inline void circularQueueRead(CircularQueue<T> *queue, T *outBuffer, unsigned int length)
{
    unsigned int length_bytes = length * queue->dataSize_bytes;
    unsigned int bytes_overflowed;
    circularQueuePeek(queue, outBuffer, length, &bytes_overflowed);

    if (bytes_overflowed == 0)
    {
        queue->readPtr += length_bytes;
    }
    else
    {
        queue->readPtr = queue->data + bytes_overflowed;
    }
}

template<typename T>
inline void circularQueuePop(CircularQueue<T> *queue, T *outBuffer, unsigned int length)
{
    unsigned int length_bytes = length * queue->dataSize_bytes;
    unsigned int bytes_overflowed;
    circularQueuePeek(queue, outBuffer, length, &bytes_overflowed);

    if (bytes_overflowed == 0)
    {
        memset(queue->readPtr, 0, length_bytes);
        queue->readPtr += length;
    }
    else
    {
        memset(queue->readPtr, 0, length_bytes - bytes_overflowed);
        memset(queue->data, 0, bytes_overflowed);
        queue->readPtr = queue->data + bytes_overflowed;
    }
}


template<typename T>
inline void circularQueuePush(CircularQueue<T> *queue, T data, unsigned int length_bytes)
{
    memcpy(queue->dataPtr, &data, length_bytes);
//    *queue->dataPtr = data;
    if (queue->dataPtr == &queue->data[queue->length - 1])
    {
        queue->dataPtr = queue->data;
    }
    else
    {
        queue->dataPtr++;
    }
}

template<typename T>
inline void circularQueuePush(CircularQueue<T> *queue, T data)
{
    circularQueuePush(queue, data, sizeof(T));
}

template<typename T>
inline bool isCircularQueueEmpty(CircularQueue<T> *queue)
{
    return queue->readPtr == queue->dataPtr;
}

#endif //HPRC_CIRCULARQUEUE_HPP
