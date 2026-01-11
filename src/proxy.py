class DobotProxy:
    def __init__(self, cmd_q, res_q):
        self._cmd_q = cmd_q
        self._res_q = res_q

    def __getattr__(self, name):
        def call(*args):
            self._cmd_q.put((name, args))
            result = self._res_q.get()   # BLOCK until worker responds

            if isinstance(result, Exception):
                raise result

            return result
        return call
