# ========================================

# Place this code at the top of your file

import pyrebase
import time

config = {
  "apiKey": "AIzaSyBCuJvm-DPjvS6P9TQ-sXhs01g76e9aWto",
  "authDomain": "ece16-fall18.firebaseapp.com",
  "databaseURL": "https://ece16-fall18.firebaseio.com",
  "storageBucket": "ece16-fall18.appspot.com",
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()

last_time = time.time()

# =========================================

# Call this function whenever you want to write to the db. Note that there is a maximum of 2 writes per second
def write_to_pyrebase(teamID, hr, steps):
    assert isinstance(teamID, str)
    assert isinstance(hr, int)
    assert isinstance(steps, int)
    
    global last_time
    current_time = time.time()
    if (current_time - last_time >= 0.5):
        last_time = current_time
        data = {"teamID": teamID, "hr": hr, "steps": steps, "timestamp": current_time}
        db.child("readings").push(data)
