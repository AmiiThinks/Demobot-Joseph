import pylab as P
import see_log as s
import GTD as g

#In this logfile, actions were the same as the last action with 95
#probability, and transitions from drive to rotation (50/50) or
#rotation to drive, or rotated on obs>0 (bumper press).
logfile1="/Users/jmodayil/data/createlogs/TwistRuns-y2014-m05-d22-h09-m31-s43.logfile"

#This second logfile has more turns, of shorter duration, with annotatins in the log.  It also doesn't backup on a bump, so they should persist more prominenetly in the log.
logfile2="/Users/jmodayil/data/createlogs/twist2/TwistRuns-y2014-m05-d22-h15-m41-s54.logfile"

labels1="netTime updateTime iteration r gamma obs act dist pktNum beh_mode  mode_duration vid_count prediction era learn_prediction use_prediction CliffL CliffR CliffFL CliffFR Distance Rotate IRbyte WheelBump DriveReq DriveCmd Voltage Current Overcurrent Button im000 im001 im002 im003 im004 im005 im006 im007 im008 im009 im010 im011 im012 im013 im014 im015 im016 im017 im018 im019 im020 im021 im022 im023 im024 im025 im026 im027 im028 im029 im030 im031 im032 im033 im034 im035 im036 im037 im038 im039 im040 im041 im042 im043 im044 im045 im046 im047 im048 im049 im050 im051 im052 im053 im054 im055 im056 im057 im058 im059 im060 im061 im062 im063 im064 im065 im066 im067 im068 im069 im070 im071 im072 im073 im074 im075 im076 im077 im078 im079 im080 im081 im082 im083 im084 im085 im086 im087 im088 im089 im090 im091 im092 im093 im094 im095 im096 im097 im098 im099".split()


def extract_data(d,name,labels=labels1):
    ind=labels.index(name)
    return d[:,ind]

def examine_binary_transistions(dataline):
    d_n=dataline[1:]>0
    d_t=dataline[:-1]>0
    net=P.sum(d_t)*1.0
    cond=P.sum(d_t*d_n)
    print "self transitions:",cond/net
    
def examine_binary_durations(dataline):
    d_n=dataline[1:]>0
    d_t=dataline[:-1]>0
    ons=P.nonzero(d_n > d_t)[0]
    offs=P.nonzero(d_n < d_t)[0]
    if len(ons) > len(offs):
        print "On at end, truncating last event"
        ons=ons[:-1]
    durations=offs-ons
    print "Events: ",len(durations)
    print "durations: min/mean/median/max",min(durations),P.mean(durations),P.median(durations),max(durations)
    return durations


def show_prediction_collision_while_driving(data):
    preds=extract_data(data,"prediction")
    beh=extract_data(data,"beh_mode")
    drive=P.array(beh==0,dtype=P.int8)
    collision=extract_data(data,"obs")>0
    P.figure()
    P.plot(drive)
    P.plot(drive*preds)
    P.plot(collision)
    revised=make_outcomes_collision_while_driving(data)
    G=P.array(revised).T
    P.plot(G[0],G[1])
#watch the data

#make one or more predictions with the original as the target.
def make_outcomes_collision_while_driving(data):
    #define r_x,rho_f,gammas_f to log
    beh=extract_data(data,"beh_mode")
    drive=P.array(beh==0,dtype=P.int8)
    collision=extract_data(data,"obs")>0
    gamma=extract_data(data,"gamma")
    R=collision[1:]
    gamma=gamma[1:]
    drive=drive[:-1]
    G_i_wt=g.compute_returns(R,gamma,drive)
    real_rets=g.restrict_returns(G_i_wt)
    return real_rets

def remake_prediction_collision_while_driving(data,gammaC=.95):
    #define r_x,rho_f,gammas_f to log
    beh=extract_data(data,"beh_mode")
    drive=P.array(beh==0,dtype=P.int8)
    collision=extract_data(data,"obs")>0
    gamma=extract_data(data,"gamma")
    R=P.array(collision,dtype=P.int8)
    gamma= gammaC*(1-R)
    N,T=1600,len(data)
    print N,T
    rawX=P.zeros((T,N),dtype=P.int8)
    for i in range(T):
        try:
            inds=[j*16 + int(data[i,-1-j]) for j in range(100)]
            rawX[i,inds]=1 #discretization
        except Exception as e:
            print e
            print "Problem at ",i
            print inds
            print data[i,-1],data[i,-100]
    print "computing"
    ps=g.compute_predictions(R,gamma,drive,rawX)
    return P.array(ps)


def make_X100(dataline):
    rawX=P.zeros((1600),dtype=P.int8)
    for j in range(100):
        rawX[j*16 + dataline[-1-j]]=1 
    return rawX
    
def learn_multiplePreds(data):
    beh=extract_data(data,"beh_mode")
    obs=extract_data(data,"obs")

    forward=g.GTD(1600,0.1/100,elambda=.95)
    turn1=g.GTD(1600,0.1/100,elambda=.95)
    turn2=g.GTD(1600,0.1/100,elambda=.95)
    
    xl=make_X100(data[0])
    preds=[]
    specs=[]
    for i in range(1,len(data)):
        xn=make_X100(data[i])
        pnow=tuple([q.predict(xn) for q in (forward,turn1,turn2)])
        spec=[]

        r=int(obs[i] > 0)
        gamma=.99*(1-r)
        rho=int(beh[i-1]==0)
        forward.update(rho,xl,r,xn,gamma)
        spec.append( (r,gamma,rho))

        E=pnow[0]
        gamma=.8
        r=(1-gamma)*E
        rho=int(beh[i-1]==1)
        turn1.update(rho,xl,r,xn,gamma)
        spec.append( (r,gamma,rho))
        
        E=pnow[0]
        gamma=.8
        r=(1-gamma)*E
        rho=int(beh[i-1]==2)
        turn2.update(rho,xl,r,xn,gamma)
        spec.append( (r,gamma,rho))

        preds.append(pnow)
        specs.append(spec)
        xl=xn
    return preds,specs
#out=cm.learn_multiplePreds(data)
#P.plot(out[0])
#Run through the data to construct returns from the specs

#Compare the data to valid returns.

#Near the end, one might find situations where pred0 is higher than
#pred1 and pred2.  These should be times when the robot was facing the
#wall, though this may not be clear from the camera's perspective in
#low resolution.  One might need an interactive interface for this to
#really kick in.  One might see this from the high resolution video,
#with p0,p1, and p2 as dancing lines.  One might use the lights to
#indicate this.  The real goal is to move from having these 3
#predictions to having ~50 that provide a sense of spatial awareness
#for the robot, so aligned images and predictions would be
#informative.  Also, in the convex environment of the pen, the middle
#prediction should never be teh lowest.  Indeed, it would be a novel
#thing to have a wooden box in the pen for the first time, from the
#perspective that it would be a conjunction of predictions that had
#never been previously experienced.  You might also use an external
#video to show the acuracy of the predictions.

#
# So what is the end message?  That it is possible to learn
# predictions from data that have amenable semantics for robots and
# sufficient semantics for people, thus reducing the burden on the
# decision maker?  Thus providing for simpler behavior specification
# and calibration with sensing modalities that are un-conventional for
# a task.

