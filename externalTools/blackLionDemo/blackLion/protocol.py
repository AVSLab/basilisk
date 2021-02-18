
class NetworkProtocol(object):
    """
        Base class to manage zmq network transport, ip, port and zmq address string

        Parameters
        ----------
        port : string
            tcp port number or ipc port name
        transport_type : string
            the transport type
    """

    def __init__(self, port=None, transport_type="tcp"):
        self._transport_type = transport_type
        self.port = port

    def get_bind_address(self):
        pass

    def get_connect_address(self):
        pass


class TcpProtocol(NetworkProtocol):
    """
        A class to manage zmq TCP network transport

        Parameters
        ----------
        host_name : string
            ip address
        port : string
            tcp port number
        address : string
            the zmq formatted tcp address string e.g. "tcp://127.0.0.1:port"
    """

    def __init__(self, host_name="127.0.0.1", port="", address=None):
        super(TcpProtocol, self).__init__(port, "tcp")
        self.ip_address = host_name
        self.port = port
        if address:
            parts = self.parse_address_parts(address)
            self.ip_address = parts["ip_address"]
            self.port = parts["port"]

    def parse_address_parts(self, address):
        parts = address.split(":")
        res = {'ip_address': None, 'port': None}
        for i in range(0, len(parts)):
            if i == 1:
                res['ip_address'] = parts[1].lstrip('/')
            elif i == 2:
                res['port'] = parts[2]
        return res

    def get_connect_address(self):
        return '{0}://{1}:{2}'.format(self._transport_type, self.ip_address, str(self.port))

    def get_bind_address(self):
        address = "{0}://{1}".format(self._transport_type, self.ip_address)
        if self.port: address += ":" + str(self.port)
        return address


class IpcProtocol(NetworkProtocol):
    """
        A class to manage zmq IPC network transport

        Parameters
        ----------
        port : string
            ipc port name
        address : string
            the zmq formatted ipc address string e.g. "ipc://port"
    """
    def __init__(self, port=None, address=None, absolute=False):
        if address:
            parts = address.split(":")
            port = parts[1].lstrip("/")
        super(IpcProtocol, self).__init__(port, "ipc")
        self.absolute = absolute

    def get_connect_address(self):
        if self.absolute:
            return '{0}:///{1}'.format(self._transport_type, self.port)
        else:
            return '{0}://{1}'.format(self._transport_type, self.port)

    def get_bind_address(self):
        if self.absolute:
            return '{0}:///{1}'.format(self._transport_type, self.port)
        else:
            return '{0}://{1}'.format(self._transport_type, self.port)
