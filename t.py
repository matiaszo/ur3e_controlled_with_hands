import datetime


hour = str(datetime.datetime.now().hour)
minute = str(datetime.datetime.now().minute)
seconds = str(datetime.datetime.now().second)

timestamp = hour + ':' + minute + ':' + seconds

print(timestamp)



# print(str(datetime.datetime.now().hour) + str(datetime.datetime.now().minute), + )
# print(datetime.datetime.now().second)