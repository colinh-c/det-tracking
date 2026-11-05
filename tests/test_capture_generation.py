"""Capture HTTP schema tests without sockets, camera processes or devices."""
import io
import json
from types import SimpleNamespace
import unittest

from pi_capture_gateway import Handler, disabled_encoders


class CaptureGenerationTests(unittest.TestCase):
    def setUp(self):
        self.info = {'sequence': 7, 'size': [1456, 1088], 'age_s': .02, 'error': None}
        self.jpeg = b'jpeg fixture bytes'
        self.readings = [{'bus': 1, 'connected': True, 'valid': False},
                         {'bus': 3, 'connected': True, 'valid': True}]
        self.server = SimpleNamespace(
            gateway_generation='capture-process-a',
            cameras=[SimpleNamespace(snapshot=lambda: (self.jpeg, self.info)) for _ in range(2)],
            encoders=SimpleNamespace(snapshot=lambda: self.readings))

    def get(self, path):
        handler = object.__new__(Handler)
        handler.server = self.server
        handler.path = path
        handler.wfile = io.BytesIO()
        status, headers = [], {}
        handler.send_response = lambda code: status.append(code)
        handler.send_header = lambda key, value: headers.__setitem__(key.lower(), value)
        handler.end_headers = lambda: None
        handler.do_GET()
        self.assertEqual(headers['x-gateway-generation'], self.server.gateway_generation)
        self.assertEqual(int(headers['content-length']), len(handler.wfile.getvalue()))
        return status[0], headers, handler.wfile.getvalue()

    def test_status_and_encoder_bodies_remain_lists(self):
        status, _, body = self.get('/status')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), [self.info, self.info])
        status, _, body = self.get('/encoders')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), self.readings)
        self.server.encoders = None
        _, _, body = self.get('/encoders')
        self.assertEqual(json.loads(body), disabled_encoders())

    def test_both_jpegs_include_same_generation_without_mutating_cached_info(self):
        original = dict(self.info)
        for path in ('/frame/0.jpg', '/frame/1.jpg'):
            status, headers, body = self.get(path)
            self.assertEqual((status, headers['content-type'], body), (200, 'image/jpeg', self.jpeg))
            metadata = json.loads(headers['x-frame-info'])
            self.assertEqual(metadata, {**original, 'gateway_generation': self.server.gateway_generation})
        self.assertEqual(self.info, original)

    def test_errors_also_identify_the_gateway_process(self):
        self.info['error'] = 'Camera feed is stale'
        status, _, body = self.get('/frame/0.jpg')
        self.assertEqual(status, 503)
        self.assertEqual(json.loads(body), self.info)
        status, _, body = self.get('/does-not-exist')
        self.assertEqual((status, body), (404, b'Not found'))

    def test_server_restart_identity_is_not_a_cached_frame_identity(self):
        self.info['gateway_generation'] = 'previous-worker-metadata'
        _, headers, _ = self.get('/frame/0.jpg')
        self.assertEqual(json.loads(headers['x-frame-info'])['gateway_generation'], 'capture-process-a')
        self.server.gateway_generation = 'capture-process-b'
        _, headers, _ = self.get('/frame/0.jpg')
        self.assertEqual(json.loads(headers['x-frame-info'])['gateway_generation'], 'capture-process-b')
        self.assertEqual(self.info['gateway_generation'], 'previous-worker-metadata')


if __name__ == '__main__':
    unittest.main()
