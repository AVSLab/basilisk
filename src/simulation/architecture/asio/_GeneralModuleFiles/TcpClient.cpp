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
// TcpClient.cpp
//
//

#include "TcpClient.h"

#ifndef MAX_CONNECT_ATTEMPTS
#define MAX_CONNECT_ATTEMPTS 5
#endif

TcpClient::TcpClient(boost::asio::io_service *ioService)
{
    m_stream.reset(new boost::asio::ip::tcp::socket(*ioService));
}

int TcpClient::connect(std::string ipAddress, uint32_t portNum)
{
    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver resolver(m_stream->get_io_service());
    boost::asio::ip::tcp::resolver::query query(ipAddress, std::to_string(portNum));
    boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query, ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return 1;
    }
    for(int attempCount = 0; attempCount < MAX_CONNECT_ATTEMPTS; attempCount++) {
        ec.clear();
        m_stream->connect(endpoint, ec);
        if(ec) {
            std::cout << "Connection attempt " << attempCount << " failed";
            if(attempCount == MAX_CONNECT_ATTEMPTS - 1) {
                std::cout << std::endl;
                std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
                close();
                m_stream = nullptr;
                return 1;
            } else {
                // Wait before attempting another connection
                std::cout << ", retrying in ";
                for(int i = 3; i > 0; i--) {
                    time_t start = time(NULL);
                    while(difftime(time(NULL), start) < 1.0);
                    std::cout << i << "...";
                    std::cout.flush();
                }
                std::cout << std::endl;
            }
        } else {
            break;
        }
    }
    // If we still can't connect then abort
    if(ec) {
        close();
        return 1;
    }
    return 0;
}

bool TcpClient::receiveData(std::vector<char> &data)
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

bool TcpClient::sendData(std::vector<char> &data)
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

bool TcpClient::close()
{

    boost::system::error_code ec;
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

void TcpClient::clearBuffers(void)
{
    m_outboundBuffer.clear();
    m_inboundBuffer.clear();
}
