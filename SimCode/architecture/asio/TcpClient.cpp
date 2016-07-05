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

int TcpClient::connect(std::string ipAddress, std::string portNum)
{
    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver resolver(m_stream->get_io_service());
    boost::asio::ip::tcp::resolver::query query(ipAddress, portNum);
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
        return 1;
    }
    return 0;
}

bool TcpClient::receiveData(std::vector<char> &data)
{
    boost::system::error_code ec;
    m_stream->read_some(boost::asio::buffer(m_inboundBuffer), ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    data = m_inboundBuffer;

    return true;
}

bool TcpClient::sendData(std::string data)
{
    boost::system::error_code ec;
    m_outboundBuffer = data;
    boost::asio::write(*m_stream, boost::asio::buffer(m_outboundBuffer), ec);
    if(ec) {
        std::cout << "Error in " << __FUNCTION__ << " (" << ec.value() << ") " << ec.message() << std::endl;
        return false;
    }
    return true;
}

void TcpClient::clearBuffers(void)
{
    m_stream->async_read_some(boost::asio::buffer(m_inboundBuffer),
                              boost::bind(&BasicIoObject_t::handleClearBuffers, this,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
}
