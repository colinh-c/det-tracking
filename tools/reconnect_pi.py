#!/usr/bin/env python3
"""Keep the existing camera/encoder dashboard linked to the Pi. No remote commands."""
from pathlib import Path
import argparse
import json
import os
import signal
import socket
import subprocess
import threading
import time

ROOT=Path(__file__).resolve().parent


def interface(host_mac=None):
    host_mac = host_mac or os.environ.get('DET_USB_HOST_MAC')
    if not host_mac:
        return None
    result=subprocess.run(['ifconfig'],capture_output=True,text=True,timeout=5)
    name=None
    for line in result.stdout.splitlines():
        if line and not line[0].isspace():name=line.split(':',1)[0]
        if line.strip().lower()=='ether '+host_mac.lower():return name
    return None


def ssh_arguments(device, pi_link_local=None, ssh_host=None, host_key_alias=None):
    address = pi_link_local or os.environ.get('DET_PI_LINK_LOCAL')
    if not address or not device:
        raise ValueError('Configure a Pi link-local address and USB interface first')
    host = ssh_host or os.environ.get('DET_SSH_HOST') or 'pi@raspberrypi.local'
    account = host.rsplit('@', 1)[0]+'@' if '@' in host else ''
    arguments = ['ssh','-N','-o','BatchMode=yes','-o','ConnectTimeout=4',
                 '-o','StrictHostKeyChecking=yes','-o','ExitOnForwardFailure=yes',
                 '-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2']
    alias = host_key_alias if host_key_alias is not None else os.environ.get('DET_SSH_HOST_KEY_ALIAS')
    if alias:
        arguments += ['-o', 'HostKeyAlias='+alias]
    return arguments+['-L','127.0.0.1:18082:127.0.0.1:8082', account+address+'%'+device]


def listening():
    with socket.socket() as connection:
        connection.settimeout(.2)
        return connection.connect_ex(('127.0.0.1',18082))==0


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once',action='store_true',help='Print the current interface and trusted tunnel command without connecting')
    parser.add_argument('--usb-host-mac', default=os.environ.get('DET_USB_HOST_MAC'), help='Known host USB Ethernet MAC, or DET_USB_HOST_MAC')
    parser.add_argument('--pi-link-local', default=os.environ.get('DET_PI_LINK_LOCAL'), help='Known Pi IPv6 link-local address without scope, or DET_PI_LINK_LOCAL')
    parser.add_argument('--ssh-host', default=os.environ.get('DET_SSH_HOST', 'pi@raspberrypi.local'), help='SSH target supplying the Pi username, or DET_SSH_HOST')
    parser.add_argument('--ssh-host-key-alias', default=os.environ.get('DET_SSH_HOST_KEY_ALIAS'), help='Optional trusted known_hosts alias, or DET_SSH_HOST_KEY_ALIAS')
    args=parser.parse_args(argv)
    missing = [name for name, value in (('DET_USB_HOST_MAC', args.usb_host_mac), ('DET_PI_LINK_LOCAL', args.pi_link_local)) if not value]
    if missing:
        print(json.dumps({'state': 'not_configured', 'missing': missing, 'interface': None, 'ssh_arguments': None}))
        return 0
    def tunnel(device):
        return ssh_arguments(device, args.pi_link_local, args.ssh_host, args.ssh_host_key_alias)
    if args.once:
        device=interface(args.usb_host_mac)
        print(json.dumps({'state': 'configured', 'interface':device, 'ssh_arguments':tunnel(device) if device else None}))
        return 0
    if listening():raise SystemExit('Camera port 18082 is already in use; preserve its existing owner')
    stop=threading.Event();process=None;last=None
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,lambda *_:stop.set())
    def report(state,**values):
        nonlocal last
        if state!=last:print(state,flush=True);last=state
        data={'state':state,'updated_unix':time.time(),'watcher_pid':__import__('os').getpid(),**values}
        temporary=ROOT/'camera-link-state.tmp';temporary.write_text(json.dumps(data,indent=2));temporary.replace(ROOT/'camera-link-state.json')
    try:
        while not stop.is_set():
            if process is not None and process.poll() is None:
                report('Camera forwarding active' if listening() else 'Connecting',tunnel_pid=process.pid)
            else:
                process=None
                if listening():
                    report('Camera port taken by another process; reconnect paused')
                    stop.wait(2);continue
                device=interface(args.usb_host_mac)
                if device is None:report('Waiting for Pi USB interface')
                else:
                    with (ROOT/'camera-link-ssh.log').open('ab') as log:
                        process=subprocess.Popen(tunnel(device),stdin=subprocess.DEVNULL,stdout=log,stderr=log)
                    report('Connecting',tunnel_pid=process.pid,interface=device)
            stop.wait(2)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=2)
        report('Camera reconnect stopped')
    return 0

if __name__=='__main__':raise SystemExit(main())
