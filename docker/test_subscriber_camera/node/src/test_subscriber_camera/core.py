class SensorLogger:
    def callback(self, data):
        print("Received %s", data)
        