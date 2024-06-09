//
// Created by William Scheirey on 3/29/24.
//

#ifndef HPRC_CIRCULARQUEUE_HPP
#define HPRC_CIRCULARQUEUE_HPP

#include <cstdlib>
#include <string>

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
    unsigned int items_left = queue->data + queue->length - queue->readPtr;
    if (items_left >= length)
    {
        memcpy(outBuffer, queue->readPtr, length * queue->dataSize_bytes);
        if (bytes_overflowed != nullptr)
        {
            *bytes_overflowed = 0;
        }
    }
    else
    {
        unsigned int bytes_until_wrap = items_left * queue->dataSize_bytes;
        unsigned int bytes_after_wrap = length * queue->dataSize_bytes - bytes_until_wrap;

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
    unsigned int bytes_overflowed;
    circularQueuePeek(queue, outBuffer, length, &bytes_overflowed);

    if (bytes_overflowed == 0)
    {
        queue->readPtr += length;
    }
    else
    {
        queue->readPtr = (T *) ((char *) queue->data + bytes_overflowed);
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
        queue->readPtr = (T *) ((char *) queue->data + bytes_overflowed);
    }
}


template<typename T>
inline void circularQueuePush(CircularQueue<T> *queue, T data)
{
    memcpy(queue->dataPtr, &data, sizeof(T));
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
inline bool isCircularQueueEmpty(CircularQueue<T> *queue)
{
    return queue->readPtr == queue->dataPtr;
}

#endif //HPRC_CIRCULARQUEUE_HPP