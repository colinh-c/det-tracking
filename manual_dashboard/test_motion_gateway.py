import importlib.util
import io
import json
import math
from pathlib import Path
import threading
import time
import unittest
from unittest import mock
from email.message import Message


SPEC = importlib.util.spec_from_file_location('motion_gateway', Path(__file__).with_name('motion_gateway.py'))
motion = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(motion)


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, amount):
        self.now += amount


class Board:
    def __init__(self):
        self.commands = []
        self.position = [0.0, 0.0]
        self.flags = 11
        self.mode = 1
        self.mres = 16
        self.bad_driver = False
        self.ambiguous_move = False
        self.never_arrive = False
        self.fail_stop = False
        self.is_open = True
        self.move_entered = None
        self.move_release = None

    def request(self, command, timeout, stop=False):
        self.commands.append(command)
        if command == 'PING':
            return ['OK PONG']
        if command == 'STAT':
            return [f'OK {self.position[0]:.5f} {self.position[1]:.5f} 0 0 0 0 {self.mode} {self.flags}']
        if command == 'DIAG':
            return [f'{axis} conn=0 toff=5 irun=12 ihold=6 mres={self.mres} spread=0 drvst=0x0 ola=1 olb=1 s2ga={int(self.bad_driver)} s2gb=0 ot=0 otpw=0 stst=1 cs=6 en_pin=0' for axis in ('AZ', 'EL')] + ['OK DIAG']
        if command == 'STOP':
            if self.fail_stop:
                raise motion.LinkError('STOP timed out')
            self.mode = 1
            return ['OK STOP']
        if command == 'ENC 0':
            self.flags &= 4
            return ['OK ENC 0']
        if command == 'MODE SLEW':
            self.mres = 16
            return ['OK MODE 16 VMAX 15.000']
        if command == 'ACC 75':
            return ['OK ACC 75.00']
        if command.startswith('MOVR '):
            if self.move_entered:
                self.move_entered.set()
                self.move_release.wait(1)
            if not self.never_arrive:
                for i, value in enumerate(command.split()[1:]):
                    self.position[i] += float(value)
            self.mode = 2
            if self.ambiguous_move:
                raise motion.LinkError('lost reply after write')
            return ['OK MOV']
        raise AssertionError(command)

    def close(self):
        self.is_open = False


class ControllerTests(unittest.TestCase):
    def setUp(self):
        logger = mock.patch('builtins.print')
        logger.start()
        self.addCleanup(logger.stop)
        self.clock = Clock()
        self.board = Board()
        self.controller = motion.MotionController(self.board, clock=self.clock.monotonic, sleep=self.clock.sleep)

    def test_no_startup_commands_and_get_reads_only_status(self):
        self.assertEqual([], self.board.commands)
        status = self.controller.status(refresh=True)
        self.assertEqual(['STAT'], self.board.commands)
        self.assertTrue(status['connected'])
        self.assertFalse(status['initialized'])
        self.assertFalse(status['encoder_feedback'])

    def test_invalid_delta_never_initializes(self):
        for delta in (True, False, None, '0.1', 0, -0.0, math.nan, math.inf, -math.inf, 10**400):
            with self.subTest(delta=delta), self.assertRaises(motion.MotionError) as caught:
                self.controller.jog('az', delta)
            self.assertEqual(400, caught.exception.status)
        self.assertEqual([], self.board.commands)

    def test_success_initializes_once_preserves_current_and_confirms_step_count(self):
        result = self.controller.jog('az', .5)
        self.assertAlmostEqual(.5, result['position']['az'], delta=16 * motion.FINE_DEG)
        self.assertEqual(.5, result['requested_delta_deg'])
        self.assertAlmostEqual(.499921875, result['commanded_delta_deg'])
        self.assertFalse(result['physical_motion_verified'])
        commands = self.board.commands
        self.assertLess(commands.index('STOP'), commands.index('MODE SLEW'))
        self.assertGreater(commands[commands.index('STOP') + 1:commands.index('ENC 0')].count('STAT'), 1)
        self.assertIn('ACC 75', commands)
        self.assertTrue(all(not c.startswith(('CLR', 'CUR', 'ZERO', 'SYNC', 'LIM', 'VEL', 'TRK', 'MOVA')) for c in commands))
        self.controller.jog('el', .25)
        self.assertEqual(1, commands.count('ENC 0'))
        self.assertEqual(1, commands.count('MODE SLEW'))

    def test_no_added_jog_or_total_travel_cap(self):
        self.controller.jog('az', 5)
        result = self.controller.jog('el', 4)
        self.assertAlmostEqual(5, result['position']['az'], delta=16 * motion.FINE_DEG)
        self.assertAlmostEqual(4, result['position']['el'], delta=16 * motion.FINE_DEG)
        self.assertNotIn('origin', result)
        self.assertNotIn('max_jog_deg', result['limits'])
        self.assertNotIn('max_total_offset_deg', result['limits'])
        self.assertEqual(75, result['limits']['acceleration_deg_s2'])

    def test_existing_firmware_limits_still_apply(self):
        self.controller.jog('az', 180)
        self.controller.jog('el', -10)
        for axis, delta in (('az', .1), ('el', -.1)):
            with self.subTest(axis=axis), self.assertRaises(motion.MotionError) as caught:
                self.controller.jog(axis, delta)
            self.assertEqual(400, caught.exception.status)
        self.assertEqual(2, sum(c.startswith('MOVR') for c in self.board.commands))
        self.controller.jog('az', -359)

    def test_quantization_preserves_counter_residue_and_stays_inside_fences(self):
        start = 9 * motion.FINE_DEG
        effective, target = motion.quantize_jog({'az': start, 'el': 0}, 'az', .1)
        self.assertEqual(9, round(target['az'] / motion.FINE_DEG) % 16)
        self.assertAlmostEqual(.09984375, effective)
        _, upper = motion.quantize_jog({'az': 0, 'el': 0}, 'az', 180)
        _, lower = motion.quantize_jog({'az': 0, 'el': 0}, 'el', -10)
        self.assertLess(upper['az'], 180)
        self.assertGreater(lower['el'], -10)

    def test_substep_request_sends_no_move_and_reports_zero_commanded_delta(self):
        result = self.controller.jog('az', .00001)
        self.assertEqual(0, result['commanded_delta_deg'])
        self.assertFalse(result['motion_command_sent'])
        self.assertFalse(result['physical_motion_verified'])
        self.assertFalse(any(command.startswith('MOVR') for command in self.board.commands))

    def test_non_grid_endpoint_would_oscillate_but_quantized_move_settles(self):
        self.board.position[1] = 4.50070
        request = self.board.request
        oscillating = [False]
        toggle = [False]
        def grid_board(command, timeout, stop=False):
            if command.startswith('MOVR'):
                requested_fine = round(float(command.split()[2]) / motion.FINE_DEG)
                oscillating[0] = requested_fine % 16 != 0
            if command == 'STOP':
                oscillating[0] = False
            if command == 'STAT' and oscillating[0]:
                toggle[0] = not toggle[0]
                self.board.position[1] = 5.5 + (16 * motion.FINE_DEG if toggle[0] else 0)
            return request(command, timeout, stop)
        self.board.request = grid_board
        result = self.controller.jog('el', 1)
        self.assertIn('MOVR 0.00000000 0.99984375', self.board.commands)
        self.assertAlmostEqual(5.500546875, result['quantized_target']['el'])
        self.assertTrue(result['step_count_target_confirmed'])

    def test_long_move_gets_distance_based_timeout(self):
        self.board.position[0] = -180
        self.board.never_arrive = True
        request = self.board.request
        move_start = []
        def delayed_move(command, timeout, stop=False):
            if command.startswith('MOVR'):
                move_start.append(self.clock.now)
            if command == 'STAT' and move_start and self.clock.now - move_start[0] >= 24.2:
                self.board.position[0] = 180
            return request(command, timeout, stop)
        self.board.request = delayed_move
        result = self.controller.jog('az', 360)
        self.assertEqual(180, result['position']['az'])
        self.assertTrue(result['step_count_target_confirmed'])
        self.assertGreater(self.clock.now - move_start[0], 24)

    def test_slew_step_quantization_is_accepted(self):
        request = self.board.request
        def quantized(command, timeout, stop=False):
            result = request(command, timeout, stop)
            if command.startswith('MOVR'):
                self.board.position[0] += 8 * motion.FINE_DEG
            return result
        self.board.request = quantized
        result = self.controller.jog('az', .1)
        self.assertTrue(result['step_count_target_confirmed'])

    def test_mode_switch_is_stopped_before_waiting_for_stable_coarse_position(self):
        self.board.position[1] = 1.50003
        request = self.board.request
        coarse_unsettled = [False]
        toggled = [False]
        def coarse_switch(command, timeout, stop=False):
            if command == 'STOP':
                if coarse_unsettled[0]:
                    self.board.position[1] = 1.49977
                coarse_unsettled[0] = False
            result = request(command, timeout, stop)
            if command == 'MODE SLEW':
                coarse_unsettled[0] = True
            if command == 'STAT' and coarse_unsettled[0]:
                toggled[0] = not toggled[0]
                self.board.position[1] = 1.49977 + (16 * motion.FINE_DEG if toggled[0] else 0)
                result = [f'OK 0 {self.board.position[1]:.5f} 0 0 0 0 1 0']
            return result
        self.board.request = coarse_switch
        result = self.controller.jog('az', .1)
        self.assertTrue(result['step_count_target_confirmed'])
        self.assertAlmostEqual(1.49977, result['position']['el'])
        first_move = next(i for i, command in enumerate(self.board.commands) if command.startswith('MOVR'))
        self.assertEqual(2, self.board.commands[:first_move].count('STOP'))

    def test_fault_flag_even_hold_and_driver_short_block_motion_without_clear(self):
        for fault_kind in ('est', 'driver'):
            with self.subTest(fault_kind=fault_kind):
                board = Board()
                board.flags = 15 if fault_kind == 'est' else 11
                board.bad_driver = fault_kind == 'driver'
                controller = motion.MotionController(board, clock=self.clock.monotonic, sleep=self.clock.sleep)
                with self.assertRaises(motion.MotionError):
                    controller.jog('az', .1)
                self.assertFalse(any(c.startswith('MOVR') for c in board.commands))
                self.assertNotIn('CLR', board.commands)

    def test_ambiguous_write_is_not_retried_and_stop_remains_available(self):
        self.board.ambiguous_move = True
        with self.assertRaises(motion.MotionError):
            self.controller.jog('az', .1)
        self.assertEqual(1, sum(c.startswith('MOVR') for c in self.board.commands))
        with self.assertRaises(motion.MotionError):
            self.controller.jog('az', .1)
        self.assertEqual(1, sum(c.startswith('MOVR') for c in self.board.commands))
        before = self.board.commands.count('STOP')
        self.controller.stop()
        self.assertEqual(before + 1, self.board.commands.count('STOP'))
        self.assertFalse(self.controller.status()['available'])

    def test_move_timeout_stops_and_latches(self):
        self.board.never_arrive = True
        with self.assertRaises(motion.MotionError):
            self.controller.jog('az', .1)
        self.assertEqual(3, self.board.commands.count('STOP'))
        self.assertFalse(self.controller.status()['available'])

    def test_reset_encoder_watchdog_reenabled_blocks_subsequent_jog(self):
        self.controller.jog('az', .1)
        self.board.flags = 11
        with self.assertRaises(motion.MotionError):
            self.controller.jog('az', .1)
        self.assertEqual(1, sum(c.startswith('MOVR') for c in self.board.commands))

    def test_reset_notice_during_initial_stop_prevents_initialization_and_move(self):
        request = self.board.request
        def with_reset(command, timeout, stop=False):
            result = request(command, timeout, stop)
            return ['# gimbal ready'] + result if command == 'STOP' else result
        self.board.request = with_reset
        with self.assertRaises(motion.MotionError):
            self.controller.jog('az', .1)
        self.assertNotIn('ENC 0', self.board.commands)
        self.assertNotIn('MODE SLEW', self.board.commands)
        self.assertFalse(any(c.startswith('MOVR') for c in self.board.commands))

    def test_stop_requires_stable_status_after_ack(self):
        request = self.board.request
        positions = iter((.02, .01, .0, .0, .0))
        def moving_stop(command, timeout, stop=False):
            if command == 'STAT':
                self.board.position[0] = next(positions, 0)
            return request(command, timeout, stop)
        self.board.request = moving_stop
        result = self.controller.stop()
        self.assertTrue(result['stopped_step_count_confirmed'])
        self.assertEqual(5, self.board.commands.count('STAT'))

    def test_shutdown_latches_before_stop_so_existing_handler_cannot_start_jog(self):
        self.controller.jog('az', .1)
        stop = self.controller.stop
        def stop_then_late_handler():
            result = stop()
            with self.assertRaises(motion.MotionError) as caught:
                self.controller.jog('el', .1)
            self.assertEqual(503, caught.exception.status)
            return result
        self.controller.stop = stop_then_late_handler
        self.controller.close()
        self.assertFalse(self.board.is_open)
        self.assertEqual(1, sum(c.startswith('MOVR') for c in self.board.commands))

    def test_busy_jog_is_rejected_and_stop_cancels_without_permanent_fault(self):
        self.board.move_entered = threading.Event()
        self.board.move_release = threading.Event()
        outcomes = []
        def jog():
            try:
                self.controller.jog('az', .1)
            except motion.MotionError as error:
                outcomes.append(error.status)
        worker = threading.Thread(target=jog)
        worker.start()
        self.assertTrue(self.board.move_entered.wait(1))
        with self.assertRaises(motion.MotionError) as caught:
            self.controller.jog('el', .1)
        self.assertEqual(409, caught.exception.status)
        stopped = []
        stopper = threading.Thread(target=lambda: stopped.append(self.controller.stop()))
        stopper.start()
        self.assertTrue(self.controller.abort.wait(1))
        self.board.move_release.set()
        worker.join(2)
        stopper.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertEqual([409], outcomes)
        self.assertTrue(stopped)
        self.assertTrue(self.controller.status()['available'])


class SerialTests(unittest.TestCase):
    class Port:
        def __init__(self, reply):
            self.reply = io.BytesIO(reply)
            self.writes = []
            self.is_open = True
            self.timeout = .01
        def write(self, value):
            self.writes.append(value)
            return len(value)
        def read_until(self, expected=b'\n', size=512):
            return self.reply.readline(size)

    def test_multiline_diagnostic_and_exact_one_write(self):
        port = self.Port(b'AZ conn=0\nEL conn=0\nOK DIAG\n')
        result = motion.SerialLink(port).request('DIAG', .1)
        self.assertEqual(3, len(result))
        self.assertEqual([b'DIAG\n'], port.writes)

    def test_unsolicited_reset_or_fault_is_not_hidden_by_ack(self):
        for text in (b'#gimbal ready\nOK MOV\n', b'! LIMIT\nOK MOV\n'):
            with self.subTest(text=text), self.assertRaises(motion.LinkError):
                motion.SerialLink(self.Port(text)).request('MOVR 0.1 0.0', .1)

    def test_stop_can_drain_stale_reply_but_keeps_notices(self):
        result = motion.SerialLink(self.Port(b'OK MOV\n! LIMIT\nOK STOP\n')).request('STOP', .8, stop=True)
        self.assertIn('! LIMIT', result)
        self.assertEqual('OK STOP', result[-1])


class HoldTests(unittest.TestCase):
    def setUp(self):
        logger = mock.patch('builtins.print')
        logger.start()
        self.addCleanup(logger.stop)
        self.board = Board()
        self.board.flags = 0
        self.controller = motion.MotionController(self.board)
        self.controller.initialized = True
        self.controller.rest_position = {'az': 0, 'el': 0}
        self.velocity_sent = threading.Event()
        request = self.board.request
        def continuous(command, timeout, stop=False):
            if command.startswith('VEL '):
                self.board.commands.append(command)
                self.board.mode = 3
                self.velocity_sent.set()
                return ['OK VEL']
            return request(command, timeout, stop)
        self.board.request = continuous
        self.addCleanup(self.controller.close)

    def test_lease_expires_without_http_polling_and_cannot_be_revived(self):
        self.controller.hold_start('lease-1', 'az', .5)
        self.assertTrue(self.velocity_sent.wait(1))
        self.controller.hold_thread.join(2)
        self.assertFalse(self.controller.hold_thread.is_alive())
        status = self.controller.status()
        self.assertFalse(status['hold_active'])
        self.assertFalse(status['recovery_required'])
        self.assertIn('STOP', self.board.commands)
        for operation in (lambda: self.controller.hold_renew('lease-1'),
                          lambda: self.controller.hold_start('lease-1', 'az', .5)):
            with self.assertRaises(motion.MotionError) as caught:
                operation()
            self.assertEqual(410, caught.exception.status)

    def test_release_before_start_tombstones_id_and_old_release_does_not_stop_new_hold(self):
        self.controller.stop(hold_id='released')
        with self.assertRaises(motion.MotionError) as caught:
            self.controller.hold_start('released', 'az', .5)
        self.assertEqual(410, caught.exception.status)
        self.controller.hold_start('new-hold', 'az', .5)
        self.assertTrue(self.velocity_sent.wait(1))
        count = self.board.commands.count('STOP')
        self.controller.stop(hold_id='released')
        self.assertEqual(count, self.board.commands.count('STOP'))
        self.assertTrue(self.controller.status()['hold_active'])
        self.controller.stop(hold_id='new-hold')
        self.controller.hold_thread.join(1)
        self.assertFalse(self.controller.status()['recovery_required'])

    def test_renew_only_current_unexpired_hold_and_fixed_jog_is_busy(self):
        self.controller.hold_start('renew', 'el', -.5)
        self.assertTrue(self.velocity_sent.wait(1))
        renewed = self.controller.hold_renew('renew')
        self.assertGreater(renewed['hold_lease_remaining_seconds'], .6)
        with self.assertRaises(motion.MotionError) as wrong:
            self.controller.hold_renew('wrong')
        self.assertEqual(410, wrong.exception.status)
        with self.assertRaises(motion.MotionError) as busy:
            self.controller.jog('az', .1)
        self.assertEqual(409, busy.exception.status)

    def test_stop_before_velocity_write_prevents_delayed_start(self):
        entered, release = threading.Event(), threading.Event()
        request = self.board.request
        def blocked_diag(command, timeout, stop=False):
            if command == 'DIAG':
                entered.set()
                release.wait(1)
            return request(command, timeout, stop)
        self.board.request = blocked_diag
        self.controller.hold_start('race', 'az', .5)
        self.assertTrue(entered.wait(1))
        stopper = threading.Thread(target=lambda: self.controller.stop(hold_id='race'))
        stopper.start()
        self.assertTrue(self.controller.abort.wait(1))
        release.set()
        stopper.join(2)
        self.controller.hold_thread.join(2)
        self.assertFalse(any(command.startswith('VEL ') for command in self.board.commands))
        self.assertFalse(self.controller.status()['recovery_required'])

    def test_ambiguous_velocity_is_not_replayed_and_fault_is_latched(self):
        request = self.board.request
        def lost_ack(command, timeout, stop=False):
            result = request(command, timeout, stop)
            if command.startswith('VEL '):
                raise motion.LinkError('VEL reply lost')
            return result
        self.board.request = lost_ack
        self.controller.hold_start('ambiguous', 'az', .5)
        self.controller.hold_thread.join(2)
        self.assertEqual(1, sum(command.startswith('VEL ') for command in self.board.commands))
        self.assertTrue(self.controller.status()['recovery_required'])
        self.assertIn('STOP', self.board.commands)

    def test_invalid_speed_and_global_stop_with_tombstone(self):
        for speed in (0, True, math.nan, math.inf, 15.01, '0.5'):
            with self.subTest(speed=speed), self.assertRaises(motion.MotionError) as caught:
                self.controller.hold_start('invalid', 'az', speed)
            self.assertEqual(400, caught.exception.status)
        self.controller.hold_start('active', 'az', .5)
        self.assertTrue(self.velocity_sent.wait(1))
        self.controller.stop(hold_id='pending', global_stop=True)
        self.controller.hold_thread.join(2)
        self.assertFalse(self.controller.status()['hold_active'])
        with self.assertRaises(motion.MotionError) as caught:
            self.controller.hold_start('pending', 'az', .5)
        self.assertEqual(410, caught.exception.status)

    def test_async_nonzero_start_can_renew_during_initialization_without_zero_or_sync(self):
        self.controller.initialized = False
        self.controller.rest_position = None
        self.board.flags = 11
        self.board.position = [-3.0, 16.0]
        entered, release = threading.Event(), threading.Event()
        request = self.board.request
        def initializing(command, timeout, stop=False):
            if command == 'MODE SLEW':
                entered.set()
                release.wait(1)
            return request(command, timeout, stop)
        self.board.request = initializing
        started = self.controller.hold_start('nonzero', 'az', .5)
        self.assertTrue(started['hold_active'])
        self.assertTrue(entered.wait(1))
        renewed = self.controller.hold_renew('nonzero')
        self.assertEqual('initializing', renewed['hold_state'])
        self.assertFalse(renewed['hold_started'])
        release.set()
        self.assertTrue(self.velocity_sent.wait(1))
        self.controller.stop(hold_id='nonzero')
        self.controller.hold_thread.join(2)
        self.assertNotIn('ZERO', self.board.commands)
        self.assertNotIn('SYNC', self.board.commands)
        self.assertIn('VEL 0.50000000 0.00000000', self.board.commands)
        self.assertFalse(self.controller.status()['recovery_required'])


class HTTPTests(unittest.TestCase):
    def request(self, path, body, headers):
        handler = motion.Handler.__new__(motion.Handler)
        handler.path = path
        handler.headers = Message()
        for key, value in headers.items():
            handler.headers[key] = value
        raw = json.dumps(body).encode()
        handler.headers['Content-Length'] = str(len(raw))
        handler.rfile = io.BytesIO(raw)
        handler.wfile = io.BytesIO()
        handler.server = type('Server', (), {'controller': motion.MotionController(Board())})()
        handler.send_response = lambda status: setattr(handler, 'code', status)
        handler.send_header = lambda *args: None
        handler.end_headers = lambda: None
        handler.do_POST()
        return handler.code, json.loads(handler.wfile.getvalue())

    def test_unauthorized_browser_posts_rejected_without_commands(self):
        for headers in ({}, {'Content-Type': 'application/json'}, {'Content-Type': 'application/json', 'X-DET-Motion': '1', 'Origin': 'https://evil.example'}, {'Content-Type': 'text/plain', 'X-DET-Motion': '1'}):
            code, _ = self.request('/motion/jog', {'axis': 'az', 'delta': .1}, headers)
            self.assertIn(code, (403, 415))

    def test_stop_supported_from_proxy_and_local_viewer(self):
        for origin in (None, 'http://127.0.0.1:18182'):
            headers = {'Content-Type': 'application/json', 'X-DET-Motion': '1'}
            if origin:
                headers['Origin'] = origin
            code, result = self.request('/motion/stop', {}, headers)
            self.assertEqual(200, code)
            self.assertTrue(result['stop_acknowledged'])


if __name__ == '__main__':
    unittest.main()
