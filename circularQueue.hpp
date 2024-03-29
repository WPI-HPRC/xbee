//
// Created by William Scheirey on 3/29/24.
//

#ifndef HPRC_CIRCULARQUEUE_HPP
#define HPRC_CIRCULARQUEUE_HPP

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

    buffer->dataPtr = &buffer->data[0];
    buffer->readPtr = buffer->dataPtr;

    return buffer;
}

template<typename T>
inline void circularQueueRead(CircularQueue<T> *queue, T *outBuffer, unsigned int length)
{
    unsigned int length_bytes = length * queue->dataSize_bytes;

    if (queue->data + queue->length - queue->readPtr > length_bytes)
    {
        memcpy(outBuffer, queue->readPtr, length_bytes);
        queue->readPtr += length_bytes;
    }
    else
    {
        unsigned int bytes_transferred = queue->data + queue->length - queue->readPtr;

        memcpy(outBuffer, queue->readPtr, bytes_transferred);
        memcpy(&outBuffer[bytes_transferred], queue->data, length_bytes - bytes_transferred);

        queue->readPtr = &queue->data[bytes_transferred];
    }
}


template<typename T>
inline void circularQueueAdd(CircularQueue<T> *buffer, T data)
{
    *buffer->dataPtr = data;
    if (buffer->dataPtr == &buffer->data[buffer->length - 1])
    {
        buffer->dataPtr = &buffer->data[0];
    }
    else
    {
        buffer->dataPtr++;
    }
}

#endif //HPRC_CIRCULARQUEUE_HPP
