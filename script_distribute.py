import sys
import os

# os.system("python mainProcess.py 0")
# os.system("python mainProcess.py 1")
# os.system("python mainProcess.py 2")
# os.system("python mainProcess.py 3")
# os.system("python mainProcess.py 4")
# os.system("python mainProcess.py 5")
# os.system("python mainProcess.py 6")
# os.system("python mainProcess.py 7")
# os.system("python mainProcess.py 8")
# os.system("python mainProcess.py 9")

import multiprocessing
import time

def func(id):
    print('you get function', id)
    os.system("python mainProcess.py "+str(id))


if __name__ == "__main__":
    pool = multiprocessing.Pool(processes=10)
    for i in range(1):
        print('range(%d)' %i)
        pool.apply_async(func, (5, ))
    pool.close()
    pool.join()
    print("Sub-process(es) done.")


