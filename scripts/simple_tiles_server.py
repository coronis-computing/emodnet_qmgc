#!/usr/bin/env python
import argparse
import os
try:
    # python 2
    from SimpleHTTPServer import SimpleHTTPRequestHandler
    from BaseHTTPServer import HTTPServer as BaseHTTPServer
except ImportError:
    # python 3
    from http.server import HTTPServer as BaseHTTPServer, SimpleHTTPRequestHandler
import SocketServer



class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept')
        self.send_header('Cache-Control', 'public,max-age=31536000')

        # Add the 'Content-Encoding: gzip' header for the terrain tiles
        path = self.translate_path(self.path)
        filename, file_extension = os.path.splitext(path)
        if file_extension == ".terrain":
            self.send_header('Content-Encoding', 'gzip')

        self.extensions_map.update({'.terrain': 'application/vnd.quantized-mesh'})
        SimpleHTTPRequestHandler.end_headers(self)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Sets up a simple http server with the CORS headers needed for serving "
                                                 "quantized mesh tiles")
    parser.add_argument("folder", type=str, default='./', help="Folder to serve")
    parser.add_argument("-p", dest="port", type=int, default=8000, help="The port the server will be listening to")
    param = parser.parse_args()

    print("Serving folder " + param.folder + " at port " + str(param.port))

    # Change the current folder
    os.chdir(param.folder)

    Handler = CORSRequestHandler
    httpd = SocketServer.TCPServer(("", param.port), Handler)

    httpd.serve_forever()