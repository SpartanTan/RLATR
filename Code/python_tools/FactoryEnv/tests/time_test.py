import time
import threading

class TimerThread(threading.Thread):
    def __init__(self):
        super(TimerThread, self).__init__()
        self.start_time = time.time()
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            elapsed_time = time.time() - self.start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            if elapsed_time % 10 <= 1:
                print(f"Elapsed time: {minutes} minutes, {seconds} seconds")
            time.sleep(1)

    def stop(self):
        self.stop_event.set()

# Then, in your main code:
timer = TimerThread()
timer.start()

k = 0
# Insert your training code here...
while True:
    k += 1
# When training is done:
timer.stop()
timer.join()


