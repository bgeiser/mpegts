#include "simple_buffer.h"
#include <assert.h>
#include <iterator>
#include <iostream>

SimpleBuffer::SimpleBuffer()
    : mPos(0)
{
}

SimpleBuffer::SimpleBuffer(int32_t size, int8_t value)
    : mPos(0)
{
    mData = std::vector<uint8_t>(size, value);
}

SimpleBuffer::~SimpleBuffer()
{
}

void SimpleBuffer::write1Byte(int8_t val)
{
    mData.push_back(val);
}

void SimpleBuffer::write2Bytes(int16_t val)
{
    char *p = (char *)&val;

    for (int i = 1; i >= 0; --i) {
        mData.push_back(p[i]);
    }
}

void SimpleBuffer::write3Bytes(int32_t val)
{
    char *p = (char *)&val;

    for (int i = 2; i >= 0; --i) {
        mData.push_back(p[i]);
    }
}

void SimpleBuffer::write4Bytes(int32_t val)
{
    char *p = (char *)&val;

    for (int i = 3; i >= 0; --i) {
        mData.push_back(p[i]);
    }
}

void SimpleBuffer::write8Bytes(int64_t val)
{
    char *p = (char *)&val;

    for (int i = 7; i >= 0; --i) {
        mData.push_back(p[i]);
    }
}

void SimpleBuffer::append(const uint8_t* bytes, int size)
{
    if (!bytes || size <= 0) {
#ifdef TSDEBUG
        std::cout << " append error " << std::endl;
#endif
        return;
    }

    mData.insert(mData.end(), bytes, bytes + size);
}

void SimpleBuffer::prepend(const uint8_t* bytes, int size)
{
    if (!bytes || size <= 0) {
#ifdef TSDEBUG
        std::cout << " prepend error " << std::endl;
#endif
        return;
    }

    mData.insert(mData.begin(), bytes, bytes + size);
}

int8_t SimpleBuffer::read1Byte()
{
    require(1);

    int8_t val = mData.at(0 + mPos);
    mPos++;

    return val;
}

int16_t SimpleBuffer::read2Bytes()
{
    require(2);

    int16_t val = 0;
    char *p = (char *)&val;

    for (int i = 1; i >= 0; --i) {
        p[i] = mData.at(0 + mPos);
        mPos++;
    }

    return val;
}

int32_t SimpleBuffer::read3Bytes()
{
    require(3);

    int32_t val = 0;
    char *p = (char *)&val;

    for (int i = 2; i >= 0; --i) {
        p[i] = mData.at(0 + mPos);
        mPos++;
    }

    return val;
}

int32_t SimpleBuffer::read4Bytes()
{
    require(4);

    int32_t val = 0;
    char *p = (char *)&val;

    for (int i = 3; i >= 0; --i) {
        p[i] = mData.at(0 + mPos);
        mPos++;
    }

    return val;
}

int64_t SimpleBuffer::read8Bytes()
{
    require(8);

    int64_t val = 0;
    char *p = (char *)&val;

    for (int i = 7; i >= 0; --i) {
        p[i] = mData.at(0 + mPos);
        mPos++;
    }

    return val;
}

std::string SimpleBuffer::readString(int len)
{
    require(len);

    std::string val(*(char*)&mData[0] + mPos, len);
    mPos += len;

    return val;
}

void SimpleBuffer::skip(int size)
{
    mPos += size;
}

void SimpleBuffer::require(int required_size)
{
    assert(required_size >= 0);

    if((unsigned int)required_size > mData.size() - mPos)
        throw std::length_error("error parsing ts: required " + std::to_string(required_size) + " bytes where " + std::to_string(mData.size() - mPos) + " are available");
}

bool SimpleBuffer::empty()
{
    return (unsigned int)mPos >= mData.size();
}

int SimpleBuffer::size()
{
    return mData.size();
}

int SimpleBuffer::pos()
{
    return mPos;
}

uint8_t* SimpleBuffer::data()
{
    return (size() == 0) ? nullptr : &mData[0];
}

void SimpleBuffer::clear()
{
    mPos = 0;
    mData.clear();
}

void SimpleBuffer::setData(int pos, const uint8_t* data, int len)
{
    if (!data) {
#ifdef TSDEBUG
        std::cout << " setData error data ptr == nullpts " << std::endl;
#endif
        return;
    }

    if (pos + len > size()) {
#ifdef TSDEBUG
        std::cout << " setData error data out of bounds " << std::endl;
#endif
        return;
    }

    for (int i = 0; i < len; i++) {
        mData[pos + i] = data[i];
    }
}

