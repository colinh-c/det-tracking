"""Watcher configuration is inspectable without network access or Pi commands."""
from contextlib import redirect_stdout
import io
import json
import os
import unittest
from unittest.mock import patch

from tools import reconnect_pi


class ReconnectConfigurationTests(unittest.TestCase):
    def test_unconfigured_watcher_reports_missing_configuration_before_any_probe(self):
        for arguments in (['--once'],):
            output = io.StringIO()
            with self.subTest(arguments=arguments), patch.dict(os.environ, {}, clear=True), patch.object(reconnect_pi.subprocess, 'run') as run, patch.object(reconnect_pi.socket, 'socket') as socket, redirect_stdout(output), patch('sys.argv', ['reconnect_pi.py', *arguments]):
                self.assertEqual(reconnect_pi.main(), 0)
            self.assertEqual(json.loads(output.getvalue()).get('state'), 'not_configured')
            run.assert_not_called()
            socket.assert_not_called()

    def test_once_matches_only_configured_mac_and_prints_strict_scoped_tunnel_without_connecting(self):
        settings = {'DET_USB_HOST_MAC': '02:00:00:00:00:02', 'DET_PI_LINK_LOCAL': 'fe80::2',
                    'DET_SSH_HOST': 'operator@bench.local', 'DET_SSH_HOST_KEY_ALIAS': 'bench-key'}
        interfaces = 'en0: flags=1\n\\tether 02:00:00:00:00:01\nen7: flags=1\n\\tether 02:00:00:00:00:02\n'.replace('\\t','\t')
        import subprocess
        output = io.StringIO()
        with patch.dict(os.environ, settings, clear=True), patch.object(reconnect_pi.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, interfaces, '')) as run, patch.object(reconnect_pi.subprocess, 'Popen') as popen, patch.object(reconnect_pi.socket, 'socket') as socket, redirect_stdout(output), patch('sys.argv', ['reconnect_pi.py', '--once']):
            self.assertEqual(reconnect_pi.main(), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result['interface'], 'en7')
        self.assertEqual(result['ssh_arguments'][-1], 'operator@fe80::2%en7')
        self.assertIn('StrictHostKeyChecking=yes', result['ssh_arguments'])
        self.assertIn('HostKeyAlias=bench-key', result['ssh_arguments'])
        self.assertEqual(run.call_args.args[0], ['ifconfig'])
        popen.assert_not_called()
        socket.assert_not_called()


if __name__ == '__main__':
    unittest.main()
