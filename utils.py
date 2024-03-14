from datetime import datetime,timedelta

def current_timestamp():
    return datetime.now()

def microseconds_since_unix_epoch_to_datetime(timestamp):
    microseconds = timestamp % 10000
    timestamp = datetime.fromtimestamp(timestamp/1000000)
    return timestamp + timedelta(microseconds=microseconds)

def datetime_to_microseconds_since_unix_epoch(dt):
    return int(dt.timestamp() * 1000000)