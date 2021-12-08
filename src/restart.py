import proto
from proto import Messenger,SerialStream

def restart():
    ser = SerialStream('/dev/ttyS0',57600)
    messenger = Messenger(ser)
    messenger.connect()
    messenger.hub.sendCommand(18)
    print("\033[92mBoard restart - done\033[00m")
    messenger.stop()
    ser.socket.close()

if __name__ == "__main__":
    restart()