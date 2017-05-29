import pylab as P
#now going up to python3

def parser(filename):  #parse create logfiles in some useful way
    f=open(filename)
    header=[]
    line=f.readline()        
    while line.split()[0]!='#Labels':
        header.append(line)
        line=f.readline()
    labels=line.split()[1:]
    print(labels)
    items=len(labels)
    raw=P.loadtxt(filename,usecols=range(items-1))
    lines=len(raw)
    line=f.readline()
    image=line.split()[-1].encode()
    imsize=len(image)
    imageN=P.zeros((lines,imsize),dtype=P.int8)
    imageN[0]=tuple(image)
    for i in range(1,lines):
        image=f.readline().split()[-1].encode()
        imageN[i,:]=tuple(image)
    f.close()

    return header, labels, raw, imageN

def plot_image_features(all,sel=range(20)):
    imageN=all[-1]
    labels=all[1]
    sensors=all[2]
    for i in range(len(sel)):
        x=sel[i]
        P.plot( 2*i + (imageN[:,i]>64), label="%d > 50 "%sel[i])
    acti=labels.index("act")
    P.plot(-sensors[:,acti]-5)

#
#import parse_logfile as pl        
#raw=pl.parser("~/data/demoBigger-y2014-m10-d31-h14-m45-s57.logfile")
#pl.plot_image_features(raw)
#


def plot_motor_delays(logfile="/Users/jmodayil/data/demowallerTest-y2015-m06-d26-h11-m34-s48.logfile"):
    header,labels,data,images=parser(logfile)
    P.figure()
    P.plot(data[:,labels.index("Distance")],label="Odometry (forward=negative)")
    P.plot(data[:,labels.index("DriveReq")],label="Last Drive Command")
    P.plot(data[:,labels.index("r")],label="Bumper Press")
    P.ylim(-3,5)
    P.legend(loc='lower right')

"""
import pylab as P
import parse_lofile as pl
pl.plot_motor_delays("/Users/jmodayil/data/demowallerTest-y2015-m06-d26-h11-m16-s19.logfile")
P.title("Sending drive commands on every timestep on Create2 (15ms)\nfile=/Users/jmodayil/data/demowallerTest-y2015-m06-d26-h11-m16-s19.logfile")
P.savefig("Slow-create2-motor-response.pdf")
"""
