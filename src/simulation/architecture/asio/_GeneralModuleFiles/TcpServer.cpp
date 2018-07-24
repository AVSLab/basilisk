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
// TcpServer.h
//
//

#include "TcpServer.h"

TcpServer::TcpServer(boost::asio::io_service *ioService)
    : m_acceptor(new boost::asio::ip::tcp::acceptor(*ioService))
{
    m_stream.reset(new boost::asio::ip::tcp::socket(*ioService));
}

bool TcpServer::acceptConnections(std::string ipAddress, uint32_t portNum)
{
    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver resolver(m_stream->get_io_service());
    boost::asio::ip::tcp::endpoint endpoint;
    bool portBound = false;
    while(!portBound)
    {
        boost::asio::ip::tcp::resolver::query query(ipAddress, std::to_string(portNum));
        endpoint = *resolver.resolve(query, ec);
        if(ec) {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }
        m_acceptor->open(endpoint.protocol(), ec);
        if(ec) {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }
        m_acceptor->set_option(boost::asio::ip::tcp::acceptor::reuse_address(false), ec);
        if(ec) {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }
        m_acceptor->bind(endpoint, ec);
        if(ec.value() == boost::system::errc::address_in_use) {
            portBound = false;
            portNum += 1;
            m_acceptor->close();
        }
        else if(ec)
        {
            std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
            return false;
        }
        else{
            portBound = true;
        }
    }

    m_acceptor->listen(boost::asio::socket_base::max_connections, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    std::cout << "Listening on: " << endpoint.port() << std::endl;

    m_acceptor->accept(*m_stream, endpoint, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_acceptor->close();
    return true;
}

bool TcpServer::close(void)
{
    boost::system::error_code ec;
    m_acceptor->close(ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    if(!m_stream)
    {
        return true;
    }
    m_stream->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    m_stream->close(ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    return true;
}

bool TcpServer::receiveData(std::vector<char> &data)
{
    boost::system::error_code ec;
	std::vector<char> dataOut;
    m_inboundBuffer = data;
    size_t dataReceived = 0;
    while(dataReceived < data.size() && isOpen() && ec.value() != boost::asio::error::eof)
    {
    	size_t dataTime = m_stream->read_some(boost::asio::buffer(m_inboundBuffer), ec);
    	dataOut.insert(dataOut.end(), m_inboundBuffer.begin(), m_inboundBuffer.begin() + dataTime);
    	dataReceived += dataTime;
    }
    if (dataReceived != data.size())
    {
    	std::cout << "Uh oh, missing data" << std::endl;
        if(ec.value() == boost::asio::error::eof)
        {
            close();
            m_stream = nullptr;
        }
    	return false;
    }
    if(ec && isOpen()) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    data = dataOut;

    return true;
}

bool TcpServer::sendData(std::vector<char> &data)
{
    boost::system::error_code ec;
    m_outboundBuffer = data;
    boost::asio::write(*m_stream, boost::asio::buffer(m_outboundBuffer), ec);
    if(ec) {
        if(ec.value() == boost::asio::error::broken_pipe)
        {
            close();
            m_stream = nullptr;
        }

        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    return true;
}

void TcpServer::clearBuffers(void)
{
    m_outboundBuffer.clear();
    m_inboundBuffer.clear();
}
