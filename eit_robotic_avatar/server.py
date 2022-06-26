from flask import Flask
from flask import send_from_directory
from flask import request

app = Flask(__name__,
            static_url_path='',
            static_folder='../www',
            )

def main():
    app.run()

@app.route('/')
def index():
    return send_from_directory('../www', 'index.html')

# This the simplest (and dumbest) way posable to implement the server side of
# WebRTC handshake and will only work under the care of the original developer.
# The server will get into a bad state on the drop of a hat.

# I just want to se WebRTC working before i do it less awful.

OFFER = {}

# Just put whatever the server send into a global variable.
# No validation no security.
# And lets hope that data that happens to be in OFFER was not important.
@app.route('/post_offer', methods=['POST'])
def put_offer():
    global OFFER
    global ANSWER
    OFFER = request.json
    ANSWER = {}
    print(OFFER)
    return {}

# Oki this just hurts.
@app.route('/get_offer', methods=['GET'])
def get_offer():
    return OFFER

# Same evil just for answers
ANSWER = {}

@app.route('/post_answer', methods=['POST'])
def put_answer():
    global OFFER
    global ANSWER
    ANSWER = request.json
    OFFER = {}
    print(ANSWER)
    return {}

@app.route('/get_answer', methods=['GET'])
def get_answer():
    return ANSWER