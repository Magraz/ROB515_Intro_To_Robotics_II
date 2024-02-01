from readerwriterlock import rwlock

class Bus:
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.msg = ""
        pass

    def write(self, msg):
        with self.lock.gen_wlock():
            self.msg = msg

    def read(self):
        with self.lock.gen_wlock():
            msg = self.msg
        return msg