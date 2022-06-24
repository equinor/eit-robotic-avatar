from flask import Flask
from flask import send_from_directory

app = Flask(__name__,
            static_url_path='', 
            static_folder='../www',
            )

def main():
    app.run(debug=True)

@app.route('/')
def index():
    return send_from_directory('../www', 'index.html')
