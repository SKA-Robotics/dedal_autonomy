#MavlinkInterface.py

class MavlinkInterface:
    def __init__(self, conn_str: str = 'udpin:0.0.0.0:14550', logger=None) -> None:
        self.master = mavutil.mavlink_connection(conn_str)
        self.master.wait_heartbeat()

        if logger is None:
            self._log = lambda lvl, msg: print(f"[{lvl.upper()}] {msg}")
        else:
            self._log = logger
        self._log('info', '✅ Połączono – heartbeat odebrany')
