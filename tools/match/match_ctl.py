#!/usr/bin/env python3
"""Send a local match command. No serial access and no velocity commands."""
import argparse
import os
import socket
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['status', 'start', 'stop', 'resume', 'finish'])
    parser.add_argument('--socket', default='/tmp/rescue-match.sock')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='rescue-match-') as directory:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
            client.bind(os.path.join(directory, 'reply.sock'))
            client.settimeout(2)
            try:
                client.sendto(args.command.encode('ascii'), args.socket)
                response = client.recv(4096).decode('utf-8')
            except (OSError, TimeoutError) as exc:
                parser.exit(1, f'Command not confirmed: {exc}\n')
            print(response)
            return 0 if response.startswith('OK ') else 1


if __name__ == '__main__':
    raise SystemExit(main())
