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
// TcpSerializeConnection.h
//
//

#ifndef TCP_SERIALIZE_CONNECTION_HPP
#define TCP_SERIALIZE_CONNECTION_HPP

#include "../_GeneralModuleFiles/BasicIoDevice.h"

#include <boost/tuple/tuple.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

class TcpSerializeConnection
    : public BasicIoObject_t<boost::asio::ip::tcp::socket>
{
public:
    TcpSerializeConnection(boost::asio::io_service *io_service)
    {
        m_stream.reset(new boost::asio::ip::tcp::socket(*io_service));
    }
    virtual ~TcpSerializeConnection() {}

    template <typename T, typename Handler>
    int receiveData(T &t, Handler handler);

    template <typename T, typename Handler>
    int sendData(const T &t, Handler handler);

    boost::asio::ip::tcp::socket *stream() { return m_stream.get(); }

private:
    // The size of the fixed length header
    enum { headerLength = 8 };

    // Outbound header
    std::string m_outputHeader;
    // Inbound header
    char m_inboundHeader[headerLength];

    template <typename T, typename Handler>
    void handleReadHeader(const boost::system::error_code &e, T &t, boost::tuple<Handler> handler);
    template <typename T, typename Handler>
    void handleReadData(const boost::system::error_code &e, T &t, boost::tuple<Handler> handler);
};

template <typename T, typename Handler>
int TcpSerializeConnection::receiveData(T &t, Handler handler)
{
    // Issue a read operation to read exactly the number of bytes in a header
    void(TcpSerializeConnection::*f)(const boost::system::error_code&, T&, boost::tuple<Handler>) =
        &TcpSerializeConnection::handleReadHeader<T, Handler>;
    boost::asio::async_read(*m_stream.get(), boost::asio::buffer(m_inboundHeader),
        boost::bind(f, this, boost::asio::placeholders::error, boost::ref(t), boost::make_tuple(handler)));
    return 0;
}

template <typename T, typename Handler>
int TcpSerializeConnection::sendData(const T &t, Handler handler)
{
    // Serialize the data first so we know how large it is
    std::ostringstream archiveStream;
    boost::archive::text_oarchive archive(archiveStream);
    try {
        archive & t;
    } catch(boost::archive::archive_exception& e) {
        std::cout << "Error occured: " << e.what() << std::endl;
        boost::system::error_code error(boost::asio::error::invalid_argument);
        return 1;
    }
//    std::cout << "archiveStream.str() " << archiveStream.str().size() << std::endl;
    uint32_t stringLength = archiveStream.str().length();
    char * cstr = new char [stringLength+1];
    std::strcpy (cstr, archiveStream.str().c_str());
    m_outboundBuffer.clear();
    m_outboundBuffer.insert(m_outboundBuffer.begin(), cstr, cstr + stringLength);

    // Format the header
    std::ostringstream headerStream;
    headerStream << std::setw(headerLength) << std::hex << m_outboundBuffer.size();
    if(!headerStream || headerStream.str().size() != headerLength) {
        // Something went wrong
        boost::system::error_code error(boost::asio::error::invalid_argument);
        m_stream->get_io_service().post(boost::bind(handler, error));
        return 1;
    }
    m_outputHeader = headerStream.str();

    // Write the serialized data to the socket
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(boost::asio::buffer(m_outputHeader));
    buffers.push_back(boost::asio::buffer(m_outboundBuffer));
    boost::asio::async_write(*m_stream.get(), buffers, handler);
    
    delete[] cstr;
    return 0;
}

// Handle a completed read of a message header. The handler is passed using
// a tuple since boost::bind seems to have trouble binding a function object
// created using boost::bind as a parameter
template <typename T, typename Handler>
void TcpSerializeConnection::handleReadHeader(const boost::system::error_code &e, T &t, boost::tuple<Handler> handler)
{
    if(e) {
        boost::get<0>(handler)(e);
    } else {
        // Determine the length of the serialized data
        std::istringstream is(std::string(m_inboundHeader, headerLength));
        size_t inboundDataSize = 0;
        if(!(is >> std::hex >> inboundDataSize)) {
            // Header doesn't seem to be valid
            boost::system::error_code error(boost::asio::error::invalid_argument);
            boost::get<0>(handler)(error);
            return;
        }

        // Start an asynchronous call to receive data
        m_inboundBuffer.resize(inboundDataSize);
        void(TcpSerializeConnection::*f)(const boost::system::error_code&, T&, boost::tuple<Handler>) = 
            &TcpSerializeConnection::handleReadData<T, Handler>;
        boost::asio::async_read(*m_stream.get(), boost::asio::buffer(m_inboundBuffer),
            boost::bind(f, this, boost::asio::placeholders::error, boost::ref(t), handler));
    }
}

// Handle a completed read of a message
template <typename T, typename Handler>
void TcpSerializeConnection::handleReadData(const boost::system::error_code &e, T &t, boost::tuple<Handler> handler)
{
    if(e) {
        boost::get<0>(handler)(e);
    } else {
        std::string archiveData(&m_inboundBuffer[0], m_inboundBuffer.size());
        std::istringstream archiveStream(archiveData);
        // Extract the data structure from the data just received
        try {
            boost::archive::text_iarchive archive(archiveStream);
            archive >> t;
            //std::cout << "Data received" << std::endl;
        } catch(boost::archive::archive_exception& e) {
            std::cout << "Error occured: " << e.what() << std::endl;
            boost::system::error_code error(boost::asio::error::invalid_argument);
            boost::get<0>(handler)(error);
            return;
        }
        // Inform caller that the data has been received ok
        boost::get<0>(handler)(e);
    }
}

#endif // TCP_SERIALIZE_CONNECTION_HPP
