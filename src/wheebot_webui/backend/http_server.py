#!/usr/bin/env python3
import json
import urllib.parse
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

from node_core import get_core

# Serve frontend/ as web root; API under /api/*
WEBROOT = 'frontend'

class Handler(SimpleHTTPRequestHandler):
    def translate_path(self, path):
        # map to frontend directory
        import os
        root = os.path.abspath(WEBROOT)
        # default to index.html on '/'
        if path == '/' or path == '':
            return os.path.join(root, 'index.html')
        # prevent path traversal
        path = path.split('?',1)[0].split('#',1)[0]
        path = path.lstrip('/')
        return os.path.join(root, path)

    def _send_json(self, obj, code=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.startswith('/api/'):
            return self.handle_api()
        return super().do_GET()

    def do_POST(self):
        if self.path.startswith('/api/'):
            return self.handle_api()
        return super().do_POST()

    def handle_api(self):
        core = get_core()
        try:
            if self.path.startswith('/api/status'):
                return self._send_json(core.get_status())
            if self.path.startswith('/api/ports'):
                return self._send_json(core.list_ports())
            if self.path.startswith('/api/teleop/wasd'):
                ln = int(self.headers.get('Content-Length','0'))
                body = self.rfile.read(ln) if ln>0 else b'{}'
                data = json.loads(body or b'{}')
                key = data.get('key','')
                return self._send_json(core.publish_wasd(key))
            if self.path.startswith('/api/stop'):
                return self._send_json(core.emergency_stop())
            if self.path.startswith('/api/nav2/goal'):
                ln = int(self.headers.get('Content-Length','0'))
                data = json.loads(self.rfile.read(ln) or b'{}')
                x = float(data.get('x',0.0)); y = float(data.get('y',0.0)); yaw = float(data.get('yaw',0.0))
                return self._send_json(core.nav2_send_goal(x,y,yaw))
            if self.path.startswith('/api/image.png'):
                qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                topic = qs.get('topic',['/camera/color/image_raw'])[0]
                png, mime = core.snapshot_image_png(topic)
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(png)))
                self.end_headers(); self.wfile.write(png); return
            if self.path.startswith('/api/map.png'):
                png, mime = core.snapshot_map_png()
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(png)))
                self.end_headers(); self.wfile.write(png); return
            if self.path.startswith('/api/pointcloud.png'):
                qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                topic = qs.get('topic',['/pointcloud/full'])[0]
                png, mime = core.snapshot_pointcloud_topdown_png(topic)
                self.send_response(200)
                self.send_header('Content-Type', mime)
                self.send_header('Content-Length', str(len(png)))
                self.end_headers(); self.wfile.write(png); return
            return self._send_json({'ok': False, 'error': 'unknown endpoint'}, 404)
        except Exception as e:
            return self._send_json({'ok': False, 'error': str(e)}, 500)


def serve(host='127.0.0.1', port=8000):
    httpd = ThreadingHTTPServer((host, port), Handler)
    print(f"WheeBot WebUI running at http://{host}:{port}")
    httpd.serve_forever()

if __name__ == '__main__':
    serve()