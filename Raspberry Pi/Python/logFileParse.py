#PARSING LOG FILE
f=open('log.txt','r+')
f1=open('reducedLog.txt', 'a+')
last=0.0

for line in f:
    try:
        if (float(line.partition(',')[0])-last>0.25):
            last = float(line.partition(',')[0])
            f1.write(line)
    except:
        pass

f.close()
f1.close()
