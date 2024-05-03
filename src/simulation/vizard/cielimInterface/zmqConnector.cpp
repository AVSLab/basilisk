/*
 ISC License

 Copyright (c) 2023 University of Colorado at Boulder

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

#include "zmqConnector.h"

ZmqConnector::ZmqConnector() = default;

void ZmqConnector::connect() {
    if (!this->isConnected()) {
        this->context = std::make_shared<zmq::context_t>();
        this->requester_socket = std::make_unique<zmq::socket_t>(*this->context, ZMQ_REQ);
        std::cout << this->comAddress << ":" << this->comPortNumber << std::endl;
        this->requester_socket->connect(this->comProtocol + "://" + this->comAddress + ":" + this->comPortNumber);
    }
    this->ping();
    this->ping();
}

bool ZmqConnector::isConnected() const {
    if (this->requester_socket) {
        return this->requester_socket->connected();
    }
    return false;
}

void ZmqConnector::send(const vizProtobufferMessage::VizMessage& message) {
    /*! - The viz needs 10 images before placing the planets, wait for 11 protobuffers to
     * have been created before attempting to go into opNavMode 2 */
    if (this->firstPass < 11){
        this->firstPass++;
    }

    /*! - send protobuffer raw over zmq_socket */
    size_t byteCount = message.ByteSizeLong();
    void* serialized_message = malloc(byteCount);
    message.SerializeToArray(serialized_message, (int)byteCount);
    auto payload = zmq::message_t(serialized_message, byteCount, ZmqConnector::message_buffer_deallocate, nullptr);

    auto emptyMsg = zmq::message_t(0);

    this->requester_socket->send(zmq::message_t("SIM_UPDATE", 10), ZMQ_SNDMORE);
    this->requester_socket->send(emptyMsg, ZMQ_SNDMORE);
    this->requester_socket->send(emptyMsg, ZMQ_SNDMORE);
    this->requester_socket->send(payload, ZMQ_NULL);

    // Receive pong
    auto pong = zmq::message_t();
    auto res = this->requester_socket->recv(&pong, ZMQ_NULL);
}

void ZmqConnector::message_buffer_deallocate(void *data, void *hint)
{
    free(data);
}

ImageData ZmqConnector::requestImage(size_t cameraId) {
    std::string cmdMsg = "REQUEST_IMAGE_";
    cmdMsg += std::to_string(cameraId);
    void* img_message = malloc(cmdMsg.length() * sizeof(char));
    memcpy(img_message, cmdMsg.c_str(), cmdMsg.length());

    auto imageRequestMessage = zmq::message_t(img_message,
                                              cmdMsg.length(),
                                              ZmqConnector::message_buffer_deallocate,
                                              nullptr);

    auto res = this->requester_socket->send(imageRequestMessage, zmq::send_flags::none);
    auto imageLengthMessage = zmq::message_t();
    auto imageMessage = zmq::message_t();
    res = this->requester_socket->recv(imageLengthMessage, zmq::recv_flags::none);
    res = this->requester_socket->recv(imageMessage, zmq::recv_flags::none);

    const int32_t *lengthPoint = imageLengthMessage.data<int32_t>();
    const void *imagePoint = imageMessage.data();
    int32_t imageBufferLength = *lengthPoint;
    void* image = malloc(imageBufferLength*sizeof(char));
    memcpy(image, imagePoint, imageBufferLength*sizeof(char));

    return  ImageData{imageBufferLength, image};
}


void ZmqConnector::ping() {
    this->requester_socket->send(zmq::message_t("PING", 4), ZMQ_NULL);
    auto message = zmq::message_t();
    auto res = this->requester_socket->recv(message, zmq::recv_flags::none);
    std::cout << message.str() << std::endl;
}
