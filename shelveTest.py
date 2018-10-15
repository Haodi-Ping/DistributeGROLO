import shelve
import datetime

info = {'name': 'bigberg', 'age': 22}
name = ['Apoll', 'Zous', 'Luna']
t = datetime.datetime.now()

with shelve.open('shelve.txt') as f:
  f['name'] = name  # 持久化列表
  f['info'] = info     # 持久化字典
  f['time'] = t      # 持久化时间类型
