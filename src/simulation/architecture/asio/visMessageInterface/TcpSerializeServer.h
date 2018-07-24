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
// TcpSerializeServer.h
//
//

#ifndef TCP_SERIALIZE_SERVER_HPP
#define TCP_SERIALIZE_SERVER_HPP

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <iostream>

#include "TcpSerializeConnection.h"

template <typename T1, typename T2>
class TcpSerializeServer
{
public:
    TcpSerializeServer();
    ~TcpSerializeServer();

    bool acceptConnections(std::string ipAddress, std::string portNum);
    bool close(void);

    void setOutboundData(T1 *data);
    T2 getInboundData();

private:
    boost::scoped_ptr<boost::asio::ip::tcp::acceptor> m_acceptor;
    boost::scoped_ptr<boost::asio::io_service> m_ioService;
    boost::scoped_ptr<boost::thread> m_thread;
    boost::mutex m_mutex;

    T1 m_outboundData;
    T2 m_inboundData;

    void handleAccept(const boost::system::error_code &e, boost::shared_ptr<TcpSerializeConnection> connection);
    void handleWrite(const boost::system::error_code &e, boost::shared_ptr<TcpSerializeConnection> connection);
    void handleRead(const boost::system::error_code &e, boost::shared_ptr<TcpSerializeConnection> connection);
};

template <typename T1, typename T2>
TcpSerializeServer<T1, T2>::TcpSerializeServer()
    : m_ioService(new boost::asio::io_service)
{

}

template <typename T1, typename T2>
TcpSerializeServer<T1, T2>::~TcpSerializeServer()
{
    close();
}

template <typename T1, typename T2>
bool TcpSerializeServer<T1, T2>::acceptConnections(std::string ipAddress, std::string portNum)
{
    if(m_thread) {
        // Thread is already running
        return true;
    }

    // Give the io_service work so it doesn't stop
    boost::asio::io_service::work work(*m_ioService.get());

    if(m_acceptor) {
        close();
    }
    m_acceptor.reset(new boost::asio::ip::tcp::acceptor(*m_ioService.get()));

    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver resolver(*m_ioService.get());
    boost::asio::ip::tcp::resolver::query query(ipAddress, portNum);
    boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_acceptor->open(endpoint.protocol(), ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_acceptor->set_option(boost::asio::ip::tcp::acceptor::reuse_address(true), ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_acceptor->bind(endpoint, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_acceptor->listen(boost::asio::socket_base::max_connections, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }

    boost::shared_ptr<TcpSerializeConnection> newConnection(new TcpSerializeConnection(m_ioService.get()));
    m_acceptor->async_accept(*newConnection->stream(), boost::bind(&TcpSerializeServer::handleAccept, this,
        boost::asio::placeholders::error, newConnection));

    // Set up the io_service to run on a separate thread so it can run truly asynchronously
    m_thread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, m_ioService.get())));

    std::cout << "Listening for new connections at " << ipAddress << ":" << portNum << std::endl;
    return true;
}

template <typename T1, typename T2>
bool TcpSerializeServer<T1, T2>::close(void)
{
    if(m_thread) {
        boost::system::error_code ec;
        m_acceptor->close(ec);
        if(ec) {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }

        m_ioService->stop();
        m_thread->join();
        m_ioService->reset();
        m_thread.reset();
    }

    return true;
}

template <typename T1, typename T2>
void TcpSerializeServer<T1, T2>::setOutboundData(T1 *data)
{
    if(m_thread) {
        boost::lock_guard<boost::mutex> guard(m_mutex);
        m_outboundData = *data;
    } else {
        m_outboundData = *data;
    }
}

template <typename T1, typename T2>
T2 TcpSerializeServer<T1, T2>::getInboundData()
{
    if(m_thread) {
        boost::lock_guard<boost::mutex> guard(m_mutex);
        return m_inboundData;
    } else {
        return m_inboundData;
    }
}

template <typename T1, typename T2>
void TcpSerializeServer<T1, T2>::handleAccept(const boost::system::error_code &ec, boost::shared_ptr<TcpSerializeConnection> connection)
{
    boost::lock_guard<boost::mutex> guard(m_mutex);
    if(!ec) {
        // Successfully accepted a new connection
        std::cout << "Successfully accepted a connection " << connection.get() <<  std::endl;
        connection->sendData(m_outboundData, boost::bind(&TcpSerializeServer::handleWrite, this,
            boost::asio::placeholders::error, connection));
    } else {
        if(ec == boost::asio::error::operation_aborted) {
            return;
        } else {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        }
    }

    // Start listening for a new connection
    boost::shared_ptr<TcpSerializeConnection> newConnection(new TcpSerializeConnection(m_ioService.get()));
    m_acceptor->async_accept(*newConnection->stream(), boost::bind(&TcpSerializeServer::handleAccept, this,
        boost::asio::placeholders::error, newConnection));
}

template <typename T1, typename T2>
void TcpSerializeServer<T1, T2>::handleWrite(const boost::system::error_code &ec, boost::shared_ptr<TcpSerializeConnection> connection)
{
    boost::lock_guard<boost::mutex> guard(m_mutex);
    if(!ec) {
        //std::cout << "Data sent to: " << connection.get() << std::endl;
        // Data sent, start listening for reply
        connection->receiveData(m_inboundData, boost::bind(&TcpSerializeServer::handleRead, this,
            boost::asio::placeholders::error, connection));
    } else {
        // Catch most common errors and replace error message with a nicer message
#ifdef WIN32
        if(ec.value() == 10054) {
#else
        if(ec.value() == 2) {
#endif
            std::cout << "Connection closed by client " << connection.get() << std::endl;
        } else {
            // An error occured
            if(ec == boost::asio::error::operation_aborted) {
                return;
            } else {
                std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            }
        }
        connection->close();
    }
}

template <typename T1, typename T2>
void TcpSerializeServer<T1, T2>::handleRead(const boost::system::error_code &ec, boost::shared_ptr<TcpSerializeConnection> connection)
{
    boost::lock_guard<boost::mutex> guard(m_mutex);
    if(!ec) {
        // Reply received, send more data
        connection->sendData(m_outboundData, boost::bind(&TcpSerializeServer::handleWrite, this,
            boost::asio::placeholders::error, connection));
    } else {
        // Catch most common errors and replace error message with a nicer message
#ifdef WIN32
        if(ec.value() == 10054) {
#else
        if(ec.value() == 2) {
#endif
            std::cout << "Connection closed by client " << connection.get() << std::endl;
        } else {
            // An error occured
            if(ec == boost::asio::error::operation_aborted) {
                return;
            } else {
                std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            }
        }
        connection->close();
    }
}

#endif // TCP_SERIALIZE_SERVER_HPP
