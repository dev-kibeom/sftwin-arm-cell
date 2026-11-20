"""Loopback-only commissioning relay. Removing 'offline' restores the same PLC."""
from contextlib import ExitStack
from select import select
from socket import create_connection, create_server
import argparse
from pathlib import Path
import time


def relay(directory, listen, plc_port):
    with create_server(('127.0.0.1', listen)) as server:
        server.settimeout(.1)
        while True:
            if (directory/'offline').exists():
                time.sleep(.05)
                continue
            try:
                peer, _ = server.accept()
            except TimeoutError:
                continue
            with ExitStack() as stack:
                stack.enter_context(peer)
                try:
                    plc = stack.enter_context(create_connection(('127.0.0.1', plc_port), timeout=1))
                    while not (directory/'offline').exists():
                        ready, _, _ = select([peer, plc], [], [], .05)
                        for source in ready:
                            data = source.recv(4096)
                            if not data:
                                raise EOFError
                            (plc if source is peer else peer).sendall(data)
                except (EOFError, OSError):
                    continue


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('exchange_dir', type=Path)
    parser.add_argument('--listen', type=int, required=True)
    parser.add_argument('--plc-port', type=int, required=True)
    args = parser.parse_args()
    relay(args.exchange_dir, args.listen, args.plc_port)
