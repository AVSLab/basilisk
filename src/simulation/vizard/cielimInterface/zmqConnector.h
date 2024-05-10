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

#ifndef ZMQCONNECTOR_H
#define ZMQCONNECTOR_H

#include "vizMessage.pb.h"
#include <zmq.hpp>
#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"

struct ImageData{
    int32_t imageBufferLength;
    void *imageBuffer;
};

class ZmqConnector {
public:
    ZmqConnector();
    ~ZmqConnector() = default;

    void connect();
    [[nodiscard]] bool isConnected() const;
    void send(const vizProtobufferMessage::VizMessage& vizMessagePayload);
    ImageData requestImage(size_t cameraId);
    void ping();

private:
    std::shared_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> requester_socket;
    int firstPass{}; //!< Flag to initialize the viz at first timestep
    std::string comProtocol = "tcp";
    std::string comAddress = "127.0.0.1";
    std::string comPortNumber = "5556";

    static void message_buffer_deallocate(void *data, void *hint);
};

#endif //ZMQCONNECTOR_H
