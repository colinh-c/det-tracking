"""Local Mac viewfinder/control service; loopback only."""
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse
import json
import cv2

PAGE=Path(__file__).with_name('dashboard.html')


def create_server(app,port=18183):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,status,data,kind='application/json'):
            self.send_response(status);self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
            self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/':self.reply(200,PAGE.read_bytes(),'text/html; charset=utf-8')
            elif path=='/status':
                with app.lock: data=json.dumps(app.status,allow_nan=False).encode()
                self.reply(200,data)
            elif path in ('/frame/narrow.jpg','/frame/wide.jpg'):
                image=app.image(path.split('/')[2].split('.')[0])
                if image is None:self.reply(503,b'{"error":"No current frame"}')
                else:self.reply(200,cv2.imencode('.jpg',image)[1].tobytes(),'image/jpeg')
            elif path=='/field_setup.html': self.reply(200,PAGE.with_name('field_setup.html').read_bytes(),'text/html; charset=utf-8')
            else:self.reply(404,b'{}')
        def do_POST(self):
            origin=self.headers.get('Origin')
            allowed='http://127.0.0.1:'+str(self.server.server_port)
            if self.headers.get('X-DET-Control')!='1' or origin not in (None,allowed):
                self.reply(403,b'{"error":"Local control header required"}');return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=16384:raise ValueError('Invalid request length')
                data=json.loads(self.rfile.read(length))
                if self.path!='/action':self.reply(404,b'{}');return
                result=app.action(data['action'],data.get('payload'))
                self.reply(200,json.dumps(result,allow_nan=False).encode())
            except (KeyError,TypeError,ValueError) as error:
                self.reply(400,json.dumps({'error':str(error)}).encode())
            except Exception as error:
                self.reply(503,json.dumps({'error':str(error)}).encode())
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    return server
