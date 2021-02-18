def convert_str_list_into_bytes(str_list):
    for i in range(len(str_list)):
        str_list[i] = str_list[i].encode("utf-8")
    return str_list


def convert_bytes_list_into_str(bytes_list):
    for i in range(len(bytes_list)):
        bytes_list[i] = bytes_list[i].decode("utf-8")
    return bytes_list
