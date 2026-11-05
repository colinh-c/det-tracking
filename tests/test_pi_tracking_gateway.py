"""Leased gateway tests. Fake only the physical serial boundary."""
import json
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from det_types import Command


class FakeLink:
    def __init__(self):
        self.is_open = True
        self.commands = []
        self.mode = 1
        self.flags = 8
        self.bad_vel = False
        self.capability = True
        self.lease_stuck = False
        self.velocity_entered = threading.Event()
        self.velocity_release = None
        self.concurrent = 0
        self.max_concurrent = 0
        self.driver_status = [0x80080000, 0x80080000]
        self.driver_enable = [0, 0]

    def request(self, command, timeout, stop=False):
        self.concurrent += 1
        self.max_concurrent = max(self.concurrent, self.max_concurrent)
        try:
            self.commands.append(command)
            if command == 'CAP':
                return ['OK CAP VEL_LEASE_V1'] if self.capability else ['ERR CMD']
            if command == 'LSTAT':
                return ['OK LSTAT %d %d %d' % (int(self.mode == 3), int(self.lease_stuck),
                                              int(time.monotonic()*1000000) & 0xFFFFFFFF)]
            if command == 'PING': return ['OK PONG']
            if command == 'STAT': return ['OK 0.00000 0.00000 0 0 0 0 %d %d' % (self.mode, self.flags)]
            if command == 'DIAG':
                return [a + ' conn=0 toff=3 mres=16 s2ga=0 s2gb=0 ot=0 otpw=0 en_pin=0 drvst=0'
                        for a in ('AZ', 'EL')] + ['OK DIAG']
            if command == 'DSTAT':
                return ['%s drvst=0x%X en_pin=%d' % (axis, status, enabled)
                        for axis, status, enabled in zip(('AZ', 'EL'), self.driver_status,
                                                         self.driver_enable)] + ['OK DSTAT']
            if command == 'STOP':
                self.mode = 1
                return ['OK STOP']
            if command == 'ENC 0':
                self.flags = 0
                return ['OK ENC 0']
            if command == 'MODE SLEW': return ['OK MODE 16 VMAX 15.000']
            if command == 'ACC 75': return ['OK ACC 75.00']
            if command.startswith('LIM '): return ['OK LIM']
            if command.startswith('LVEL '):
                self.velocity_entered.set()
                if self.velocity_release is not None:
                    self.velocity_release.wait(1)
                self.mode = 3
                fields = command.split()
                return ['OK LVEL %s %s %s' % (fields[1], fields[2], fields[3])] if not self.bad_vel else ['OK MOV']
            raise AssertionError(command)
        finally:
            self.concurrent -= 1

    def close(self):
        self.is_open = False


class GatewayTests(unittest.TestCase):
    def setUp(self):
        from pi_tracking_gateway import TrackingGateway
        self.link = FakeLink()
        self.gateway = TrackingGateway(self.link, start_watchdog=False)
        self.gateway.initialize()

    def tearDown(self):
        self.gateway.close()

    def payload(self, sequence=1, ttl=.2, **updates):
        value = dict(timestamp=5000000.0, valid_until=5000000.25, enabled=True,
                     reason='tracking', sequence=sequence, az_velocity_deg_s=-.5,
                     el_velocity_deg_s=.3, ttl_s=ttl,
                     generation=self.gateway.status()['generation'])
        value.update(updates)
        return value

    def test_initialization_and_bidirectional_velocity_are_acknowledged(self):
        status = self.gateway.velocity(self.payload())
        self.assertTrue(status['active'])
        sent = next(x.split() for x in self.link.commands if x.startswith('LVEL '))
        self.assertEqual(sent[-2:], ['-0.50000000', '0.30000000'])
        self.assertTrue(0 < int(sent[2]) <= 200)
        self.assertTrue(status['firmware_watchdog'])
        self.assertFalse(any(c.startswith('VEL ') for c in self.link.commands))
        self.assertFalse(any(c.startswith(('CLR', 'CUR', 'ZERO', 'SYNC', 'MOV')) for c in self.link.commands))
        self.assertEqual(self.link.max_concurrent, 1)

    def test_duplicate_stale_and_nonfinite_commands_never_reach_serial(self):
        from pi_tracking_gateway import GatewayError
        self.gateway.velocity(self.payload())
        count = len([c for c in self.link.commands if c.startswith('LVEL ')])
        for payload in [self.payload(), self.payload(2, ttl=0), self.payload(3, az_velocity_deg_s=float('nan')),
                        self.payload(4, az_velocity_deg_s=16), self.payload(sequence=True)]:
            with self.assertRaises(GatewayError):
                self.gateway.velocity(payload)
        self.assertEqual(len([c for c in self.link.commands if c.startswith('LVEL ')]), count)

    def test_lease_expiry_stops_and_invalidates_old_generation(self):
        from pi_tracking_gateway import GatewayError
        payload = self.payload(ttl=.015)
        self.gateway.velocity(payload)
        time.sleep(.02)
        self.gateway.poll()
        status = self.gateway.status()
        self.assertFalse(status['active'])
        self.assertEqual(status['last_reason'], 'lease_expired')
        self.assertNotEqual(status['generation'], payload['generation'])
        payload['sequence'] += 1
        with self.assertRaises(GatewayError): self.gateway.velocity(payload)

    def test_new_command_cannot_revive_an_expired_lease_before_watchdog_poll(self):
        from pi_tracking_gateway import GatewayError
        self.gateway.velocity(self.payload(ttl=.01))
        old_generation = self.gateway.status()['generation']
        time.sleep(.015)
        with self.assertRaises(GatewayError):
            self.gateway.velocity(self.payload(2))
        self.assertFalse(self.gateway.status()['active'])
        self.assertNotEqual(self.gateway.status()['generation'], old_generation)
        self.assertEqual(len([c for c in self.link.commands if c.startswith('LVEL ')]), 1)

    def test_stop_wins_over_velocity_in_flight_and_queued_commands(self):
        from pi_tracking_gateway import GatewayError
        old = self.payload()
        self.link.velocity_release = threading.Event()
        failures = []
        def send():
            try: self.gateway.velocity(old)
            except GatewayError as error: failures.append(error)
        sender = threading.Thread(target=send)
        sender.start()
        self.assertTrue(self.link.velocity_entered.wait(.5))
        stopper = threading.Thread(target=self.gateway.stop)
        stopper.start()
        limit = time.monotonic()+.3
        while self.gateway.status()['generation'] == old['generation'] and time.monotonic() < limit:
            time.sleep(.001)
        self.assertNotEqual(self.gateway.status()['generation'], old['generation'])
        self.link.velocity_release.set()
        sender.join(1); stopper.join(1)
        self.assertFalse(sender.is_alive() or stopper.is_alive())
        self.assertFalse(self.gateway.status()['active'])
        old['sequence'] += 1
        with self.assertRaises(GatewayError): self.gateway.velocity(old)
        motor_commands = [x for x in self.link.commands if x == 'STOP' or x.startswith('LVEL ')]
        self.assertEqual(motor_commands[-1], 'STOP')
        self.assertEqual(self.link.max_concurrent, 1)

    def test_bad_ack_and_controller_fault_latch(self):
        from pi_tracking_gateway import GatewayError
        self.link.bad_vel = True
        with self.assertRaises(GatewayError): self.gateway.velocity(self.payload())
        self.assertIsNotNone(self.gateway.status()['fault'])
        with self.assertRaises(GatewayError): self.gateway.velocity(self.payload(2))
        self.assertFalse(self.gateway.status()['active'])

    def test_controller_fault_is_checked_even_when_mode_claims_track(self):
        from pi_tracking_gateway import GatewayError
        self.gateway.velocity(self.payload())
        self.link.flags = 4
        with self.assertRaises(GatewayError):
            self.gateway.velocity(self.payload(2))
        self.assertIsNotNone(self.gateway.status()['fault'])
        # Preserve the fault latch but let teardown independently verify STOP.
        self.link.flags = 0

    def test_watchdog_thread_expires_without_any_further_http_requests(self):
        from pi_tracking_gateway import TrackingGateway
        self.gateway.close()
        self.link = FakeLink()
        self.gateway = TrackingGateway(self.link)
        self.gateway.initialize()
        self.gateway.velocity(self.payload(ttl=.025))
        end = time.monotonic()+.5
        while self.gateway.status()['active'] and time.monotonic() < end:
            time.sleep(.005)
        self.assertFalse(self.gateway.status()['active'])
        self.assertEqual(self.gateway.status()['last_reason'], 'lease_expired')

    def test_transport_alias_host_is_accepted_but_cross_origin_is_not(self):
        from pi_tracking_gateway import make_server
        server = make_server(self.gateway, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:%d' % server.server_port
        try:
            # SSH forwarding preserves the Mac-side Host port, not Pi's 8084.
            with urlopen(Request(url+'/status', headers={'Host':'127.0.0.1:18084'}), timeout=.5) as response:
                self.assertTrue(json.load(response)['initialized'])
            for extra in ({'Host':'evil.example'}, {'Origin':'https://evil.example'}):
                headers = {'Content-Type':'application/json', 'X-DET-Motion':'1', **extra}
                with self.assertRaises(HTTPError) as error:
                    urlopen(Request(url+'/stop', data=b'{}', headers=headers), timeout=.5)
                self.assertEqual(error.exception.code, 403)
        finally:
            server.shutdown(); server.server_close(); thread.join(1)

    def test_http_motor_roundtrip_stop_and_header_requirement(self):
        from pi_tracking_gateway import make_server
        from transport import HttpMotor, TransportError
        server = make_server(self.gateway, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = 'http://127.0.0.1:%d' % server.server_port
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(url+'/velocity', data=b'{}', headers={'Content-Type':'application/json'}), timeout=.5)
            self.assertEqual(error.exception.code, 403)
            motor = HttpMotor(url, timeout_s=.05)
            now = time.monotonic()
            motor.send(Command(now, .25, -.1, now+.25, True, sequence=9))
            self.assertTrue(self.gateway.status()['active'])
            motor.stop()
            with self.assertRaises(TransportError):
                now = time.monotonic()
                motor.send(Command(now, 1, 0, now+.25, True, sequence=10))
            self.assertFalse(self.gateway.status()['active'])
        finally:
            server.shutdown(); server.server_close(); thread.join(1)

    def test_unused_motor_client_cannot_acquire_a_new_token_after_external_stop(self):
        from pi_tracking_gateway import make_server
        from transport import HttpMotor, TransportError
        server = make_server(self.gateway, port=0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            motor = HttpMotor('http://127.0.0.1:%d' % server.server_port, timeout_s=.05)
            self.gateway.stop()
            now = time.monotonic()
            with self.assertRaises(TransportError):
                motor.send(Command(now, .2, 0, now+.25, True, sequence=1))
            self.assertFalse(any(c.startswith('LVEL ') for c in self.link.commands))
        finally:
            server.shutdown(); server.server_close(); thread.join(1)

    def test_legacy_firmware_is_refused_without_any_motion_or_configuration(self):
        from pi_tracking_gateway import TrackingGateway, GatewayError
        self.gateway.close()
        self.link = FakeLink()
        self.link.capability = False
        self.gateway = TrackingGateway(self.link, start_watchdog=False)
        with self.assertRaises(GatewayError): self.gateway.initialize()
        self.assertFalse(self.gateway.status()['initialized'])
        self.assertFalse(self.gateway.status()['firmware_watchdog'])
        self.assertFalse(any(c.startswith(('VEL ', 'LVEL ', 'MODE ', 'ENC ', 'ACC ')) for c in self.link.commands))

    def test_stop_requires_firmware_lease_latch_clear_not_just_stable_counts(self):
        from pi_tracking_gateway import GatewayError
        self.link.lease_stuck = True
        with self.assertRaises(GatewayError):
            self.gateway.stop()
        self.assertIsNotNone(self.gateway.status()['fault'])
        self.link.lease_stuck = False


class FastDriverStatusTests(unittest.TestCase):
    def make_gateway(self):
        from pi_tracking_gateway import TrackingGateway
        clock = [100.]
        def sleep(duration):
            clock[0] += duration
        link = FakeLink()
        gateway = TrackingGateway(link, clock=lambda: clock[0], sleep=sleep,
                                  start_watchdog=False)
        gateway.initialize()
        self.addCleanup(gateway.close)
        return gateway, link, clock

    @staticmethod
    def velocity(gateway, sequence=1):
        return gateway.velocity(dict(timestamp=5000., valid_until=5000.5, ttl_s=.5,
                                     az_velocity_deg_s=.1, el_velocity_deg_s=-.1,
                                     enabled=True, sequence=sequence, reason='test',
                                     generation=gateway.status()['generation']))

    def test_runtime_health_keeps_half_second_cadence_without_full_diagnostics(self):
        gateway, link, clock = self.make_gateway()
        self.assertEqual(link.commands.count('DIAG'), 2)
        self.assertEqual(link.commands.count('DSTAT'), 1)  # require protocol before motion
        self.velocity(gateway)
        clock[0] += .49
        gateway.poll()
        self.assertEqual(link.commands.count('DSTAT'), 1)
        self.velocity(gateway, 2)
        clock[0] += .06  # status poll is also due
        gateway.poll()
        self.assertTrue(gateway.status()['active'])
        self.assertEqual(link.commands.count('DIAG'), 2)
        self.assertEqual(link.commands.count('DSTAT'), 2)
        self.assertEqual(link.max_concurrent, 1)

    def test_fresh_pose_is_new_serial_sample_and_cannot_suppress_driver_health(self):
        gateway, link, clock = self.make_gateway()
        self.velocity(gateway)
        before = link.commands.count('STAT')
        value = gateway.fresh_pose()
        self.assertEqual(link.commands.count('STAT'), before+1)
        self.assertEqual(value['generation'], gateway.status()['generation'])
        self.assertEqual(value['position'], {'az': 0., 'el': 0.})
        self.assertGreaterEqual(value['age_s'], 0)
        gateway.last_diag = clock[0]-.51
        gateway.fresh_pose()  # last_status is now, but health must still run
        gateway.poll()
        self.assertEqual(link.commands.count('DSTAT'), 2)

    def test_fresh_pose_rejects_busy_serial_without_returning_cached_position(self):
        from pi_tracking_gateway import GatewayError
        gateway, link, _ = self.make_gateway()
        before = len(link.commands)
        with gateway.io_lock, self.assertRaises(GatewayError): gateway.fresh_pose()
        self.assertEqual(len(link.commands), before)

    def test_earlier_leased_firmware_without_fast_status_cannot_initialize(self):
        from pi_tracking_gateway import TrackingGateway, GatewayError
        class EarlierFirmware(FakeLink):
            def request(self, command, timeout, stop=False):
                if command == 'DSTAT':
                    self.commands.append(command)
                    return ['ERR CMD']
                return super().request(command, timeout, stop)
        link = EarlierFirmware()
        clock = [100.]
        def sleep(duration): clock[0] += duration
        gateway = TrackingGateway(link, clock=lambda: clock[0], sleep=sleep,
                                  start_watchdog=False)
        self.addCleanup(gateway.close)
        with self.assertRaises(GatewayError):
            gateway.initialize()
        self.assertFalse(gateway.status()['initialized'])
        self.assertIsNotNone(gateway.status()['fault'])
        self.assertFalse(any(command.startswith('LVEL ') for command in link.commands))

    def test_each_short_and_temperature_fault_bit_latches_and_stops_each_axis(self):
        from pi_tracking_gateway import GatewayError
        for axis in range(2):
            for bit in range(6):  # OTPW, OT, S2GA, S2GB, S2VSA, S2VSB
                with self.subTest(axis=axis, bit=bit):
                    gateway, link, clock = self.make_gateway()
                    self.velocity(gateway)
                    generation = gateway.status()['generation']
                    link.driver_status[axis] |= 1 << bit
                    gateway.last_diag = clock[0] - .51
                    gateway.last_status = clock[0] - .06
                    gateway.poll()
                    self.assertIsNotNone(gateway.status()['fault'])
                    self.assertFalse(gateway.status()['active'])
                    self.assertNotEqual(gateway.status()['generation'], generation)
                    self.assertEqual([c for c in link.commands if c == 'STOP' or c.startswith('LVEL ')][-1], 'STOP')
                    with self.assertRaises(GatewayError):
                        self.velocity(gateway, 2)

    def test_invalid_read_and_disabled_driver_latch_at_runtime(self):
        for axis in range(2):
            for raw, enable in ((0, 0), (0xFFFFFFFF, 0), (0x80080000, 1)):
                with self.subTest(axis=axis, raw=raw, enable=enable):
                    gateway, link, clock = self.make_gateway()
                    self.velocity(gateway)
                    link.driver_status[axis] = raw
                    link.driver_enable[axis] = enable
                    gateway.last_diag = clock[0] - .51
                    gateway.last_status = clock[0] - .06
                    gateway.poll()
                    self.assertIsNotNone(gateway.status()['fault'])
                    self.assertFalse(gateway.status()['active'])

    def test_fast_status_wire_parser_rejects_ambiguous_and_malformed_data(self):
        import pi_tracking_gateway as module
        validator = getattr(module, 'validate_driver_status', None)
        self.assertTrue(callable(validator), 'Fast status parser is required')
        healthy = ['AZ drvst=0x800800C0 en_pin=0', 'EL drvst=0x40080000 en_pin=0', 'OK DSTAT']
        validator(healthy)  # open-load/standstill flags are not short/temperature faults
        bad_lines = [
            healthy[:2], healthy + ['OK DSTAT'],
            [healthy[1], healthy[0], healthy[2]],
            ['AZ drvst=0x80080000 en_pin=0 en_pin=1', healthy[1], healthy[2]],
            ['AZ drvst=0x100000000 en_pin=0', healthy[1], healthy[2]],
            ['AZ drvst=-1 en_pin=0', healthy[1], healthy[2]],
            ['AZ drvst=0x80080000 en_pin=2', healthy[1], healthy[2]],
            ['AZ drvst=0x80080000', healthy[1], healthy[2]],
            ['AZ drvst=0x80080000 en_pin=0 extra=0', healthy[1], healthy[2]],
            [healthy[0], healthy[1], 'OK DIAG'],
        ]
        for lines in bad_lines:
            with self.subTest(lines=lines), self.assertRaises(module.LinkError):
                validator(lines)

    def test_serial_transaction_accepts_only_the_fast_status_response_shape(self):
        from pi_tracking_gateway import SerialLink, LinkError
        class Port:
            is_open = True
            def __init__(self, replies):
                self.replies = list(replies)
                self.writes = []
            def write(self, payload):
                self.writes.append(payload)
                return len(payload)
            def read_until(self, ending, size):
                return self.replies.pop(0) if self.replies else b''
        expected = ['AZ drvst=0x80080000 en_pin=0', 'EL drvst=0x80080000 en_pin=0', 'OK DSTAT']
        port = Port([(line+'\n').encode() for line in expected])
        self.assertEqual(SerialLink(port).request('DSTAT', .1), expected)
        self.assertEqual(port.writes, [b'DSTAT\n'])
        with self.assertRaises(LinkError):
            SerialLink(Port([b'OK DIAG\n'])).request('DSTAT', .1)


class FirmwareLeaseTests(unittest.TestCase):
    def test_actual_sketch_fast_status_reads_once_and_preserves_motion_and_lease(self):
        import pathlib
        import shutil
        import subprocess
        import tempfile
        compiler = shutil.which('clang++') or shutil.which('g++')
        self.assertIsNotNone(compiler)
        root = pathlib.Path(__file__).resolve().parents[1]
        source = r'''
        #include "Arduino.h"
        #include "hardware/structs/sio.h"
        #include <cassert>
        uint32_t g_millis = 0;
        int g_pinState[64];
        SerialStub Serial, Serial1, Serial2;
        static sio_hw_t s_sio;
        sio_hw_t *sio_hw = &s_sio;
        #include "gimbal_skr_pico.ino"
        static void command(const char *text) {
            char line[80]; strcpy(line, text); handleLine(line);
        }
        int main() {
            az.pos = 12345; el.pos = -6789;
            az.velCmd = 15; el.velCmd = -12;
            gMode = MODE_TRACK;
            // Normal, active and expired lease states all allow observation.
            for (int state=0; state<3; ++state) {
                gVelocityLease.active = state==1;
                gVelocityLease.tripped = state==2;
                gVelocityLease.last_sequence = 99;
                gVelocityLease.deadline_us = 123456;
                drvAZ.status = 0xFFFFFFFFu; drvEL.status = 0x8008003Fu;
                g_pinState[AZ_EN] = 0; g_pinState[EL_EN] = 1;
                unsigned beforeAZ=drvAZ.status_reads, beforeEL=drvEL.status_reads;
                command("DSTAT");
                assert(drvAZ.status_reads == beforeAZ+1);
                assert(drvEL.status_reads == beforeEL+1);
                assert(gVelocityLease.active == (state==1));
                assert(gVelocityLease.tripped == (state==2));
                assert(gVelocityLease.last_sequence == 99 && gVelocityLease.deadline_us == 123456);
                assert(gMode == MODE_TRACK && az.pos == 12345 && el.pos == -6789);
                assert(az.velCmd == 15 && el.velCmd == -12);
                command("DSTAT extra");
                assert(drvAZ.status_reads == beforeAZ+1 && drvEL.status_reads == beforeEL+1);
            }
        }
        '''
        with tempfile.TemporaryDirectory(prefix='det-dstat-test-') as tmp:
            cpp, binary = pathlib.Path(tmp)/'status.cpp', pathlib.Path(tmp)/'status'
            cpp.write_text(source)
            result = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                                     '-Wno-unused-function', '-Wno-unused-parameter',
                                     '-I', str(root/'tests/firmware_stubs'),
                                     '-I', str(root/'firmware/gimbal_skr_pico'),
                                     str(cpp), '-o', str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_native_lease_deadline_sequence_wrap_and_braking(self):
        import pathlib
        import shutil
        import subprocess
        import tempfile
        compiler = shutil.which('clang++') or shutil.which('g++')
        self.assertIsNotNone(compiler, 'A C++ compiler is required to verify the firmware lease')
        root = pathlib.Path(__file__).resolve().parents[1]
        source = r'''
        #include "det_velocity_lease.h"
        #include <cassert>
        #include <cmath>
        int main() {
            DetVelocityLease l;
            assert(!l.accept(0, 0, 1000, 1000));
            assert(!l.accept(0, 501, 1000, 1000));
            assert(l.accept(0, 100, 1000000, 1000000));
            assert(!l.accept(0, 100, 1050000, 1050000)); // duplicate cannot refresh
            assert(!l.tick(1099999));
            assert(l.tick(1100000));
            assert(l.tripped && !l.active);
            assert(!l.accept(1, 100, 1100000, 1100000));
            l.resetAfterStop();
            assert(l.accept(1, 1, 0xfffffff0u, 0xfffffff0u));
            assert(!l.tick(0x000003d7u));
            assert(l.tick(0x000003d8u)); // unsigned clock wrap
            l.resetAfterStop();
            assert(l.accept(5, 50, 1000, 1000));
            assert(!l.accept(4, 50, 2000, 2000));
            assert(!l.accept(6, 50, 51000, 51000)); // expiry before ISR poll
            assert(l.tripped);
            l.resetAfterStop();
            assert(l.accept(0, 100, 1000, 1000));
            l.beginStop();
            assert(!l.accept(1, 100, 2000, 2000));
            l.resetAfterStop();
            assert(!l.accept(0, 50, 1000, 51000)); // delayed serial delivery
            assert(!l.active && !l.tripped);
            assert(!l.accept(0, 50, 2000, 1000)); // future reference
            assert(l.accept(0, 50, 1000, 2000));
            assert(l.deadline_us == 51000); // not 52000: delivery adds no life
            float p=15, n=-15;
            for(int i=0;i<201;i++) { p=detLeaseBrake(p,.075f); n=detLeaseBrake(n,.075f); }
            assert(p==0 && n==0);
            assert(detLeaseBrake(.01f,.075f)==0);
            assert(detLeaseBrake(-.01f,.075f)==0);
        }
        '''
        with tempfile.TemporaryDirectory(prefix='det-lease-test-') as tmp:
            cpp = pathlib.Path(tmp)/'lease.cpp'
            binary = pathlib.Path(tmp)/'lease'
            cpp.write_text(source)
            result = subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(root/'firmware/gimbal_skr_pico'), str(cpp), '-o', str(binary)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
