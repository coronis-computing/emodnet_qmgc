#!/usr/bin/env python3
"""Serves a folder of quantized-mesh tiles over HTTP, with the headers Cesium needs (CORS and gzip encoding)."""
import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class CORSRequestHandler(SimpleHTTPRequestHandler):
    extensions_map = dict(SimpleHTTPRequestHandler.extensions_map)
    extensions_map.update({'.terrain': 'application/vnd.quantized-mesh'})

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept')
        self.send_header('Cache-Control', 'no-cache')

        # The tiles are stored gzipped by qm_tiler
        if self.path.split('?', 1)[0].endswith('.terrain'):
            self.send_header('Content-Encoding', 'gzip')

        super().end_headers()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Sets up a simple http server with the CORS headers needed for serving "
                                                 "quantized mesh tiles")
    parser.add_argument("folder", type=str, nargs='?', default='./', help="Folder to serve")
    parser.add_argument("-p", dest="port", type=int, default=8000, help="The port the server will be listening to")
    param = parser.parse_args()

    print("Serving folder " + param.folder + " at port " + str(param.port))

    # Cesium requests many tiles at once, so serve them from several threads
    handler = functools.partial(CORSRequestHandler, directory=param.folder)
    httpd = ThreadingHTTPServer(("", param.port), handler)
    httpd.serve_forever()
