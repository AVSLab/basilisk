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
// TcpSerializeClient.h
//
//

#ifndef TCP_SERIALIZE_CLIENT_HPP
#define TCP_SERIALIZE_CLIENT_HPP

#include "TcpSerializeConnection.h"

template <typename T1, typename T2>
class TcpSerializeClient
{
public:
    TcpSerializeClient();
    ~TcpSerializeClient();

    // Starts asynchronous connect, only returns false if an immediate error occurs
    bool connect(std::string ipAddress = "127.0.0.1", std::string portNum = "50000");
    // Check whether the client is connected
    bool isConnected();

    virtual bool close(void);
    // Calls any outstanding socket handlers (connect/read/write/etc). Used instead
    // of run to control how often things are updated and to avoid unnecessary threads
    void poll();

    T1 getInboundData();
    void setOutboundData(T2 *data);

private:
    boost::scoped_ptr<boost::asio::io_service> m_ioService;
    boost::scoped_ptr<TcpSerializeConnection> m_connection;
    std::string m_ipAddress;
    std::string m_portNum;
    T1 m_inboundData;
    T2 m_outboundData;
    bool m_isAttemptingConnection;
    bool m_shouldReconnect;

    void handleConnect(const boost::system::error_code &e);
    void handleRead(const boost::system::error_code &e);
    void handleWrite(const boost::system::error_code &e);

    bool close(bool shouldReconnect);
    void reconnect();
};

template <typename T1, typename T2>
TcpSerializeClient<T1, T2>::TcpSerializeClient()
    : m_ioService(new boost::asio::io_service)
    , m_connection(new TcpSerializeConnection(m_ioService.get()))
    , m_isAttemptingConnection(false)
    , m_shouldReconnect(false)
{

}

template <typename T1, typename T2>
TcpSerializeClient<T1, T2>::~TcpSerializeClient()
{
    close();
}

template <typename T1, typename T2>
bool TcpSerializeClient<T1, T2>::connect(std::string ipAddress, std::string portNum)
{
    if(!m_isAttemptingConnection) {
        if(m_connection->isOpen()) {
            close();
        }
        m_ipAddress = ipAddress;
        m_portNum = portNum;
        boost::system::error_code ec;
        boost::asio::ip::tcp::resolver resolver(*m_ioService);
        boost::asio::ip::tcp::resolver::query query(ipAddress, portNum);
        boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query, ec);
        if(ec) {
            // An error occurred
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            m_isAttemptingConnection = false;
            return false;
        }
        // Start an asynchronous connect operation
        m_connection->stream()->async_connect(endpoint, boost::bind(&TcpSerializeClient::handleConnect, this,
            boost::asio::placeholders::error));
        m_isAttemptingConnection = true;
        m_shouldReconnect = true;
        m_ioService->poll();
    }
    return true;
}

template <typename T1, typename T2>
bool TcpSerializeClient<T1, T2>::isConnected()
{
    return m_connection->isOpen() && !m_isAttemptingConnection;
}

template <typename T1, typename T2>
bool TcpSerializeClient<T1, T2>::close()
{
    return close(false);
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::poll()
{
    if(m_connection->isOpen()) {
        m_ioService->poll();
    }
}

template <typename T1, typename T2>
T1 TcpSerializeClient<T1, T2>::getInboundData()
{
    return m_inboundData;
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::setOutboundData(T2 *data)
{
    m_outboundData = *data;
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::handleConnect(const boost::system::error_code &ec)
{
    if(!ec) {
        m_isAttemptingConnection = false;
        // Successfully established a connection
        std::cout << "Successfully established a connection" << std::endl;
        m_connection->receiveData(m_inboundData, boost::bind(&TcpSerializeClient::handleRead, this, boost::asio::placeholders::error));
        m_ioService->poll();
    } else {
        // Catch most common errors and replace error message with a nicer message
#ifdef WIN32
        if(ec.value() == 10061) {
#else
        if(ec.value() == 61) {
#endif
            std::cout << "Server not responding." << std::endl;
        } else {
            // An error occurred
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        }
        reconnect();
    }
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::handleRead(const boost::system::error_code &ec)
{
    if(!ec) {
        // Received data, send reply
        m_connection->sendData(m_outboundData, boost::bind(&TcpSerializeClient::handleWrite, this, boost::asio::placeholders::error));
    } else {
        // Catch most common errors and replace error message with a nicer message
        if(ec == boost::asio::error::eof || ec.value() == 10054) {
            std::cout << "Connection closed by server" << std::endl;
        } else if(ec == boost::asio::error::invalid_argument) {
            // An error occured while trying to read the data
        } else {
            // An error occured
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        }
        reconnect();
    }
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::handleWrite(const boost::system::error_code &ec)
{
    if(!ec) {
        // Reply sent, start listening
        m_connection->receiveData(m_inboundData, boost::bind(&TcpSerializeClient::handleRead, this, boost::asio::placeholders::error));
    } else {
        // Catch most common errors and replace error message with a nicer message
        if(ec == boost::asio::error::eof || ec.value() == 10054) {
            std::cout << "Connection closed by server" << std::endl;
        } else {
            // An error occured
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        }
        reconnect();
    }
}

template <typename T1, typename T2>
bool TcpSerializeClient<T1, T2>::close(bool shouldReconnect)
{
    m_shouldReconnect = shouldReconnect;
    m_isAttemptingConnection = false;
    if(shouldReconnect) {
        // If we are reconnecting only close socket
        return m_connection->close();
    } else {
        // Reset the connection to null first since it depends on the io_service
        m_connection.reset();
        m_ioService->stop();
        m_ioService.reset(new boost::asio::io_service);
        // Now that the io_service has been reset, reset the connection
        m_connection.reset(new TcpSerializeConnection(m_ioService.get()));
        return true;
    }
}

template <typename T1, typename T2>
void TcpSerializeClient<T1, T2>::reconnect()
{
    if(m_shouldReconnect) {
        std::cout << "Attempting to reconnect..." << std::endl;
        close(true);
        connect(m_ipAddress, m_portNum);
    } else {
        close(false);
    }
}

#endif // TCP_SERIALIZE_CLIENT_HPP