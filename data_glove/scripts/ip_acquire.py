#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : ip_acquire.py
@Author  : Li_JiaXuan
@Date    : 2025-1-18 20:10
@Software: PyCharm
"""
import socket
from typing import Optional


def get_local_ip() -> Optional[str]:
    """
    Retrieve the local IP address of the machine.

    Returns:
        Optional[str]: The local IP address if successfully retrieved;
                       None if an error occurs.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            local_ip = sock.getsockname()[0]

        if not local_ip:
            raise ValueError("Local IP address could not be retrieved.")

        return local_ip

    except Exception as error:
        print(f"Error retrieving local IP address: {error}")
        raise SystemExit("Exiting program due to error.")


if __name__ == "__main__":
    ip_address = get_local_ip()
    print(f"The local IP address is: {ip_address}")
