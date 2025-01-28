from http.server import BaseHTTPRequestHandler, HTTPServer
import os
import wavToText

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_POST(self):
        """Receives a file via HTTP POST."""
        try:
            # Get the file name from the headers (if available)
            content_disposition = self.headers.get('Content-Disposition')
            if content_disposition:
                _, params = cgi.parse_header(content_disposition)
                filename = params['filename']
            else:
                filename = 'received_file.wav'  # Default filename

            # Create the file path
            file_path = os.path.join('uploads', filename) 

            # Create the 'uploads' directory if it doesn't exist
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            # Read the file content
            file_length = int(self.headers.get('Content-Length'))
            with open(file_path, 'wb') as f:
                f.write(self.rfile.read(file_length))

            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'File received successfully!\n')
            wav_file_path = "uploads/received_file.wav" 
            transcribed_text = wavToText.transcribe_wav(wav_file_path)
            print(transcribed_text)

        except Exception as e:
            self.send_error(500, 'Error receiving file: {}'.format(str(e)))

def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler, port=8000):
    """Starts the HTTP server."""
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Starting httpd on port {}'.format(port))
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()

if __name__ == '__main__':
    run()
