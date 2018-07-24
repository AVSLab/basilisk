/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
//
// BasicIoDevice.h
//
//

#ifndef BASIC_IO_DEVICE_H
#define BASIC_IO_DEVICE_H

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <iostream>

#ifndef STREAM_BUFFER_SIZE
#define STREAM_BUFFER_SIZE 65507
#endif

/*---------------------------------------------------------------------------*/
class BasicIoObject
{
public:
    BasicIoObject() {}

    virtual ~BasicIoObject() {}

    virtual bool isOpen(void) = 0;
    virtual bool close(void) = 0;

    virtual bool receiveData(std::vector<char> &data) = 0;
    virtual bool sendData(std::vector<char> &data) = 0;

    virtual void clearBuffers(void) = 0;

};

/*---------------------------------------------------------------------------*/
template<typename StreamType>
class BasicIoObject_t : public BasicIoObject
{
public:
    BasicIoObject_t();
    virtual ~BasicIoObject_t();

    virtual bool isOpen(void);
    virtual bool close(void);

    virtual bool receiveData(std::vector<char> &data) { return 1; }
    virtual bool sendData(std::vector<char> &data) { return 1; }
    virtual uint64_t bytesWaiting() {return m_stream->available();}

    std::string getInputBuffer() { return m_inboundBuffer; }
    virtual void clearBuffers(void) {}
    void handleClearBuffers(const boost::system::error_code &ec, size_t bytes_transferred);
    void appendToOutbound(const char * newBytes, uint64_t byteSize);
    boost::shared_ptr<StreamType> m_stream;

protected:
    
    std::vector<char> m_outboundBuffer;
    std::vector<char> m_inboundBuffer;
};

/*---------------------------------------------------------------------------*/
template<typename StreamType>
BasicIoObject_t<StreamType>::BasicIoObject_t()
    : BasicIoObject()
{
    m_inboundBuffer.clear();
    m_outboundBuffer.clear();
}

template<typename StreamType>
BasicIoObject_t<StreamType>::~BasicIoObject_t()
{
    close();
}

template<typename StreamType>
bool BasicIoObject_t<StreamType>::isOpen()
{
    if(m_stream) {
        return m_stream->is_open();
    } else {
        return false;
    }
}

template<typename StreamType>
bool BasicIoObject_t<StreamType>::close(void)
{
    if(isOpen()) {
        boost::system::error_code ec;
        m_stream->close(ec);
        if(ec) {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }
        m_stream = nullptr;
        m_inboundBuffer.clear();
        m_outboundBuffer.clear();
    }
    return true;
}

template<typename StreamType>
void BasicIoObject_t<StreamType>::handleClearBuffers(const boost::system::error_code &ec,
        size_t bytesReceived)
{
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
    }
}

template <typename StreamType>
void BasicIoObject_t<StreamType>::appendToOutbound(const char *newBytes, uint64_t byteSize)
{
    m_outboundBuffer.insert(m_outboundBuffer.end(), newBytes, newBytes+byteSize);
}

#endif
