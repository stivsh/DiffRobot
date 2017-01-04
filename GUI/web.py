from flask import Flask
from flask import render_template
app = Flask(__name__)

@app.route('/')
def hello_world():
    return render_template('index.html')

@app.route('/pad/<string:command>')
def show_user_profile(command):
    # show the user profile for that user
    return 'command %s' % command
