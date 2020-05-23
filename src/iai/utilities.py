DEBUG = True
import binascii

def debug_print(string):
    if DEBUG:
        print(string)


def wait(condition):
    while condition():
        pass

def retry(func, retry_amount):
    while(True):
        try:
            r = func()
            return r
        except:
            retry_amount = retry_amount - 1
            if retry_amount <= 0:
                raise

def calculate_check_sum(source_data):
    check_sum = 0
    for i in source_data:
        check_sum += ord(i)
    check_sum &= 0xff
    return check_sum

def hex_to_int_list(hex_string):
    hex_data = binascii.unhexlify(hex_string)
    try:
        return list(map(ord, hex_data))
    except:
        return list(map(int, hex_data))

def int_to_upper_hex(int_number, length=None):
    int_number = int(int_number)
    byte = hex(int_number)
    byte = byte.replace("0x", "")
    if length:
        if len(byte) < length:
            byte = "0" * (length - len(byte)) + byte
        if len(byte) > length:
            byte = byte[-length:]
    else:
        if len(byte) % 2 == 1:
            byte = "0" + byte
    return byte.upper()


class SimpleAsyncWorker(object):
    def __init__(self):
        try:
                from queue import Queue
        except ImportError:
                from Queue import Queue

        import threading
        self.q = Queue()
        self.thread = threading.Thread(target=self.worker)
        self.thread.daemon = True
        self.thread.start()

    def worker(self):
        while True:
            new_job = self.q.get()
            new_job()
            self.q.task_done()

    def add_job(self, function, callback=None, *args, **kwargs):
        if callback:
            self.q.put(lambda: callback(function(*args, **kwargs)))
        else:
            self.q.put(lambda: function(*args, **kwargs))

    def join(self):
        self.q.join()

