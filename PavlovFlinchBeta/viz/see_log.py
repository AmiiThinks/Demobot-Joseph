#visualize images from logfiles

import pylab as P
import yuv 

Roffsets=[11049, 762, 28773, 6970, 21827, 16422, 10899, 22019, 3608, 34002, 34436, 33649, 7659, 35809, 22383, 22378, 12723, 7753, 29159, 2642, 33939, 34839, 16440, 12830, 451, 15998, 21677, 36795, 25323, 11949, 10673, 26031, 32362, 28244, 28983, 37918, 10359, 10897, 22487, 8772, 33205, 25344, 8735, 13493, 21716, 15370, 3324, 35946, 26235, 38134, 19140, 21498, 2768, 15987, 15133, 28747, 12062, 5839, 19553, 1625, 14781, 12984, 21374, 19385, 7350, 18197, 23097, 14495, 35811, 20564, 20571, 7781, 17033, 13293, 26657, 30509, 33114, 34005, 43, 19859, 14354, 28634, 27113, 22894, 18172, 19826, 3389, 3143, 8098, 34368, 13399, 20070, 36407, 4291, 25395, 26469, 28814, 1849, 7253, 32620] 

yuvd={"size":(120,160,2),"count":100}

logfile="../logfiles/Waller-y2014-m01-d17-h17-m41-s48.logfile"

def getoffsetpixels(logfile,linenumber):
    f=open(logfile)
    for i in range(linenumber):
        d=f.readline()
    vals=[int(x) for x in d.split()[-yuvd["count"]:]]
    return vals

def precompute_offsets(size=(120,160)):
    off=Roffsets
    lookup=yuv.makelookups(off,size)
    return lookup


#I should look into a TK interactive view from the vals.
def plotoffsetvals(vals,lookup,size=(120,160),offsets=Roffsets): 
    #Plot the centers in R,G,B
    rescaled=[16*v for v in vals]
    yuvimg=yuv.fill_offset_vals(rescaled,lookup,size)
    rgb=yuv.rgbFromyuv422(yuvimg)
    for p in offsets:
        q=p%4
        x=(p/2) %160
        y=p/(2*160)
        r,g,b=0,0,0
        if q==0 or q==2:
            r=255
        if q==1:
            g=255
        if q==3:
            b=255
        rgb[y,x, 0]=r
        rgb[y,x, 1]=g
        rgb[y,x, 2]=b
    yuv.plotrgbdata(rgb)

#set date
#take pictures with yuvtest
#see teh discretized image


#---------------------------------------------
##
#
#
# Analyze a log to get the plots of interest

def read_log1(fname):
    d=P.loadtxt(fname,skiprows=1)
    N=len(d[0])
    vfeats=list(range(N-100,N+1))
    netTime=0
    iteration=1
    r=2
    gamma=3
    obs=4
    act=5 
    dist=6
    pktNum=7 
    beh_mode=8
    mode_duration=9
    vid_count=10 
    prediction=11

def get_labels(fname):
    f=open(fname)
    f.readline()
    f.readline()
    f.readline()
    labels=f.readline().split()[1:]
    return labels

def make_rets(rs,gammas):
    lastG=0
    out=[]
    for i in range(len(rs)-1,0,-1):
        G=rs[i] + gammas[i]*lastG
        lastG=G
        out.append(G)
    out.reverse()
    return out


def make_rho_rets(rs,gammas,rhos):
    lastG=0
    out=[]
    for i in range(len(rs)-1,0,-1):
        if gammas[i]==0:
            G=rs[i]
        else:
            G=rs[i] + gammas[i]*lastG
        if rhos[i-1]:
            lastG=G
            out.append(G)
        else:
            lastG=P.nan
            out.append(P.nan)
    out.reverse()
    return out


def show_paths(d,labels,XMIN=0,XMAX=60,XSTEP=10,RUN_START=0,RHO_RETS=False):
    f,ax=pre_plot()
    R=labels.index("r")
    Gamma=labels.index("gamma")
    Prediction=labels.index("prediction")
    Beh_mode=labels.index("beh_mode")
    rs=d[:,R]
    gammas=d[:,Gamma]
    beh_mode=d[:,Beh_mode]
    if (RHO_RETS):
        rets=make_rho_rets(rs,gammas,(beh_mode==0) +(beh_mode==3))
    else:
        rets=make_rets(rs,gammas)
    #dist=d[:,6]/20
    preds=d[:,Prediction]
    P.plot(preds,"r")
    #P.plot(gammas,"g")
    P.plot(rets,"b")
    #P.plot(dist,"b")
    P.ylim(-.5,1.5)
    x=list(range(len(d)))
    ax.fill_between(x,0,1, where= P.logical_not((beh_mode==0)+(beh_mode==3)),edgecolor='none',facecolor='black',alpha=0.9,zorder=10)
    ax.fill_between(x,-.1,0, where= (beh_mode==3),edgecolor='none',facecolor='green',alpha=0.9,zorder=10)
    ax.fill_between(x,1,1.1, where= rs!=0 ,facecolor='grey',edgecolor='none',alpha=0.9,zorder=10)
    duration=33.3333
    P.xlim((RUN_START+XMIN)*duration,(RUN_START+XMAX)*duration)
    xns=P.arange(XMIN,XMAX,XSTEP)
    ax.set_xticks([(RUN_START+i)*duration for i in xns ])
    ax.set_xticklabels(["%d"%i for i in xns])

    P.xlabel("Running time (seconds)")
    P.ylim(-.1,1.1)
    post_plot(f,ax)
    return rets,preds


def grab_onsets(d,l):
    #get the in-order indices of reward onsets
    rind=l.index("r")
    rs=d[:,rind]
    onsets=[i for i in range(1,len(d)) if rs[i-1]==0 and rs[i]==1]
    return onsets

#def grab_explore_onsets(d,rind=3,beh_ind=9):
#    #get the in-order indices of reward onsets
#    rs=d[:,rind]
#    explore=P.where(d[:,beh_ind]==3)
#    onsets=[i for i in range(1,len(d)) if rs[i-1]==0 and rs[i]==1]
#    return onsets
def grab_explore_onsets(d,labels):
    bind=labels.index("beh_mode")
    b=d[:,bind]==3
    b=b*1
    return 1+P.where(b[1:]-b[:-1] >0)[0]

def make_cumulative_bump_distribution(d,bb,l):
    rind=l.index("r") #bump
    outs=[]
    for b in bb:
        v=34
        try:
            for i in range(34):
                if d[b+i,rind]==1:
                    v=i
                    break
        except:
            print("Exception in make_cumulative_bump_distribution")
        outs.append(v)
    return outs

import math

def evaluate_mean_return_to_bump_from_explore(bump_times):
    q=[math.pow(.95,x) for x in bump_times]
    return P.mean(q),P.std(q)

def evaluate_from_explore_onsets(d,l):
    ons=grab_explore_onsets(d,l)
    print(ons)
    pind=l.index('prediction')
    pred_distr=P.mean(d[ons,pind]),P.std(d[ons,pind])
    bump_times=make_cumulative_bump_distribution(d,ons,l)
    nn=[x for x in bump_times if x < 34]
    q=len(ons),len(nn)
    print(q)
    perf1=evaluate_mean_return_to_bump_from_explore(nn) 
    perf2=evaluate_mean_return_to_bump_from_explore(bump_times) 
    return "total explore, explore collides within 1 sec",q, "\n return mean,sig from 1 seconds",perf1,"\n return mean,sig from total",perf2, "\n relative error bounds ",float(len(bump_times)-len(nn))/len(bump_times),"\n prediction distribution",pred_distr
#In [440]: s.evaluate_from_explore_onsets(d1,l1)
#Out[440]: (0.62399694692878127, 0.11627906976744186)

def extract_from_explore_bumps(d,l):
    ons=grab_explore_onsets(d,l)
    bump_times=make_cumulative_bump_distribution(d,ons,l)
    nn=[o+x for o,x in zip(ons,bump_times) if x < 34]
    pp=l.index("prediction")
    raw= [d[n-40:n,pp] for n in nn]
    mn=P.mean([b for b in raw],axis=0)
    P.plot(list(range(-40,0)), mn)
    P.plot(list(range(-40,0)),[math.exp(x/20.) for x in range(-40,0)])
    return raw
#raw=s.extract_from_explore_bumps(d1,l1)
#P.savefig("aligned-explore-bump-prediction-returns.pdf")


def find_change_onsets(d,index): 
    return P.where(0!=d[:-1,index]-d[1:,index])[0]
    
def collate_eras_collisions(d,labels,rind):
    ERA=labels.index("era")
    USE=labels.index("use_prediction")
    R=rind#    R=labels.index("r")
    eras=[-1,]+list(find_change_onsets(d,ERA))+[len(d)-1,]
    print(eras)
    Ps,NPs=[],[]
    for i in range(len(eras)-1):
        start,end=1+eras[i],eras[i+1]+1
        print(i,start,end)
        data=P.cumsum(d[start:end,R])
        print(data.shape)
        p=d[start+1,USE]==1
        if p:
            Ps+=[(P.arange(start,end),data)]
        else:
            NPs+=[(P.arange(start,end),data)]
    return Ps,NPs

def plot_runs(p,np):
    f,ax=pre_plot()
    for x,y in p:
        P.plot(x,y,"b")
    for x,y in np:
        P.plot(x,y,"r")
    post_plot(f,ax)
def plot_overlap(p,np):
    f,ax=pre_plot()
    for x,y in p:
        P.plot(y,"b")
    for x,y in np:
        P.plot(y,"r")
    post_plot(f,ax)
def add_significance(x_ys1,x_ys2,plot_sigs):
        #if the differences are significant at the .001 level, then report
        out=""
        for i in range(len(x_ys1)):
            x1,ys1=x_ys1[i]
            x2,ys2=x_ys2[i]
            if x1!=x2:
                print("error x1!=x2",x1,x2)
                return "error x1!=x2"
            else:
                v=report_significance(ys1,ys2)
                print(x1,v)
                out+=" %s %s\n"%(x1,v)
                if (v< .001 and plot_sigs):
                    P.plot(x1,P.amax(ys2),"k*")
        return out

def plot_means(p,np,k=5800,XM=180,XS=30,YM=60,YS=10,ymult=30#33.333
               ,ylabel=None,fixys=True,plot_sigs=True):
    f,ax=pre_plot()
    def aggregate(ys,col="r",step=k/10):
        ys=[l[:k] for l in ys if len(l) > k]
        means=P.mean(ys,axis=0)
        print(len(means))
        xs = list(range(step,k,step))
        mus=[means[x] for x in xs]
        den=P.sqrt(len(ys))
        stds=[ P.std([y[x] for y in ys])/den for x in xs]
        sigs=[(x, [y[x] for y in ys]) for x in xs]
        ax.plot(means,col)
        ax.errorbar(xs,mus,stds,ecolor=col, fmt=None)        
        return means,sigs
    mp,sigp=aggregate([y for x,y in p],"b")
    np,signp=aggregate([y for x,y in np],"r")
    out=add_significance(signp,sigp,plot_sigs)
    post_plot(f,ax)
    xns=list(range(0,XM,XS))
    #33.33
    P.xlim(0,XM)
    ax.set_xticks([i*33.33 for i in xns ])
    ax.set_xticklabels(["%d"%i for i in xns])
    if fixys:
        yns=list(range(0,YM,YS))
        ax.set_yticks([i*ymult for i in yns ])
        ax.set_yticklabels(["%d"%i for i in yns]) 
    P.xlabel("Running time (seconds)")
    if ylabel==None:
        ylabel="Total Collision Time (seconds)"
    P.ylabel(ylabel)#,rotation="horizontal",horizontalalignment='center')
    return mp,np,out



def pre_plot():
    P.matplotlib.rcParams.update({'font.size': 20, 'pdf.use14corefonts' : True, 'ps.useafm':True, 'font.serif':['Times New Roman']})
    f=P.figure(figsize=(8,5))
    ax=f.add_subplot(111)
    return f,ax

def post_plot(f,ax):
    #remove excess ink and clean up plots
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    ax.get_yaxis().set_tick_params(direction='out')
    ax.get_xaxis().set_tick_params(direction='out')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)


    #ax1.set_xticks(xs)
    #ax1.get_xaxis().tick_bottom()
    #ax1.get_yaxis().tick_left()
    #ys=[0, 20000, 40000, 60000]#range(0,1100*80,200*80)
    #ax1.set_yticks(ys)
    #ax1.set_yticklabels(["0","20,000","40,000","60,000"]) 
    #choose reasonable axis labels


def grab_data_labels_nofeats(filename):
    ls=get_labels(filename)
    last=ls.index("im000")
    print(last,ls[last])
    d=P.loadtxt(filename,usecols=list(range(0,last)))
    return d,ls[:last]

def make_aligned(d,onsets,l):
    pind=l.index("prediction")
    preds=d[:,pind]
    p=[P.mean([preds[i+o] for o in onsets]) for i in range(-100,0)]
    s=[P.std([preds[i+o] for o in onsets]) for i in range(-100,0)]
    xs=[]
    for o in onsets:
        X=0
        xr=[]
        for i in range(-100,0):        
            x=max(X,preds[i+o])
            X=x
            xr.append(x)
        xs.append(xr)
    mean=P.mean(xs,axis=0)
    med=P.median(xs,axis=0)
    Lo=P.percentile(xs,2,axis=0)
    lo=P.percentile(xs,20,axis=0)
    hi=P.percentile(xs,80,axis=0)
    Hi=P.percentile(xs,98,axis=0)
    g=[pow(.95,-i) for i in range(-100,0)]
    xs=list(range(-100,0))
    f,ax=pre_plot()
    ax.plot(xs,mean,"r")
    ax.plot(xs,med,"g")
    #P.plot(xs,lo)
    #P.plot(xs,hi)
    #P.plot(xs,Lo)
    #P.plot(xs,Hi)
    #P.errorbar(xs,p,yerr=s)
    ax.plot(xs,g,"b")
    post_plot(f,ax)
    P.xlabel("Timesteps before bump")
    P.ylabel("Prediction")

def grab_nonpavlovian_bump_onsets(d,l,val=0):
    mode=l.index("use_prediction")
    #eind=l.index("era")
    q=grab_onsets(d,l)
    q=[x for x in q if d[x,mode]==val]
    return q
#nonpavs=s.grab_nonpavlovian_bump_onsets(d1,l1)
#s.make_aligned(d1,nonpavs,l1)
#P.savefig("Aligned-predictions-nonpavlovian.pdf")

#pavs=s.grab_nonpavlovian_bump_onsets(d1,l1)
#s.make_aligned(d1,pavs,l1)
#P.savefig("Aligned-predictions-pavlovian.pdf")

#d1=P.loadtxt("./Waller-y2014-m01-d27-h17-m02-s41.logfile")
#
#

#d=P.loadtxt("./Waller-y2014-m01-d21-h10-m58-s42.logfile",skiprows=1)
#s.show_paths(d)
#P.savefig("Pavlovian-Collision-predictions-individual.pdf")
#ons=s.grab_onsets(d)
#s.make_aligned(d,ons[20:])
#P.savefig("Pavlovian-Collision-predictions-distribution.pdf")
#
# I should show what fraction of responses have been triggered within
# t of the onset. (a distribution of triggered responses)
#
# We could also show the prediction that minimizes the least-squares
# error from data (fixed optimal in the feature space).
#


# I need the performance graph for different behaviour policies.
# Here, it would be distance/collisions or distance/time?  I guess
# previously, I showed this via driving efficiency and over-current
# observations in time.


def cumulative_bumps_over_time(d,bump_obs=4):
    return  P.cumsum(d[:,bump_obs]!=0)


import bisect
def cumulative_bumps_over_time(ds,step=30):
    net=[]
    for d in ds:
        net.append( (d[:,0],cumulative_bumps_over_time(d)))
    time=[]
    all=[]
    for t in range(step,300,step):        
        vals=[]
        for ts,cs in range(len(net)):
            k=bisect.bisect(ts,t)
            vals.append(cs[k])
        time.append(t)
        all.append(vals)
    return time,all

#show means and sigmas for data files
#
#
#

def report_significance(valsA,valsB):
    #do two sets of values have the same mean? 
    #returns the p-value 
    #eg. stats.ttest_ind(P.random(50),P.random(50)+1)[1]
    import scipy.stats as ss
    return ss.ttest_ind(valsA,valsB)[1]


#d0,l0=s.grab_data_labels_nofeats("./Waller-y2014-m01-d26-h21-m58-s47.logfile")
#rind=l0.index("r")
#p,np=s.collate_eras_collisions(d0,l0,rind)
#s.plot_means(p,np)
#P.savefig("downstairs-errors.pdf")
#dind=l0.index("dist")
#p,np=s.collate_eras_collisions(d0,l0,dind,ylabel="Accumulated Travel Distance")
#s.plot_means(p,np,ylabel="Distance travelled (meters)",fixys=True,YM=25,YS=5,ymult=1000)
#P.savefig("downstairs-progress.pdf")
#
#d1,l1=s.grab_data_labels_nofeats("./Waller-y2014-m01-d27-h17-m02-s41.logfile")
#rind=l1.index("r")
#p,np=s.collate_eras_collisions(d1,l1,rind)
#s.plot_means(p,np,k=9600, XM=330,YM=25,YS=5)
#P.savefig("pen-errors.pdf")
#dind=l1.index("dist")
#p,np=s.collate_eras_collisions(d1,l1,dind)
#s.plot_means(p,np,ylabel="Distance travelled (meters)",fixys=True,YM=25,YS=5,ymult=1000,k=9600, XM=330)
#P.savefig("pen-progress.pdf")


experiment_results="/Users/jmodayil/data/pavlov/"
run013113="Waller-y2014-m01-d31-h12-m00-s43.logfile"
run013114="Waller-y2014-m01-d31-h13-m35-s02.logfile"
run013116="Waller-y2014-m01-d31-h15-m17-s42.logfile"
run013117="Waller-y2014-m01-d31-h16-m17-s44.logfile"
run0131night="Waller-y2014-m01-d31-h18-m31-s19.Logfile"
run0201nightoffice="Waller-y2014-m02-d01-h21-m59-s37.logfile"

ar="/Users/jmodayil/data/pavlovCreate/"
ar12=ar+"Waller-y2014-m04-d14-h12-m17-s28.logfile"
ar16=ar+"Waller-y2014-m04-d14-h16-m36-s33.logfile"
ar17=ar+"Waller-y2014-m04-d14-h17-m17-s28.logfile"
ar20=ar+"Waller-y2014-m04-d14-h20-m17-s28.logfile"

fixedapr25=ar+"Waller-y2014-m04-d15-h01-m17-s29.logfile"
fixedapr26=ar+"Waller-y2014-m04-d26-h16-m19-s44.logfile"


#clip last lines from these files from the command line.
#sed -i '' -e '$ d' Foo.txt

def check_log(filename,checked=False):
    if not checked:
        print("sed -i '' -e '$ d' "+filename)
        print("check_log("+filename+",True)")
        return
    print(filename)
    di,li=grab_data_labels_nofeats(filename)
    aa,bb,cc=start_end(di,li)
    d=di[aa:bb]
    l=li
    return d,l


def evaluate_reactive_predictions(preds,rets,data,labels):
    # select the predictions and returns that correspond to 
    # reactive control and driving forward.
    # it explains about 73% of the signal variance.
    reactind=labels.index('use_prediction')
    react=data[:,reactind]==0
    driveind=labels.index('beh_mode')
    drive=data[:,driveind]==0
    z=react * drive
    T=z[:-1]
    print(sum(T))
    rr=P.array(rets)
    pp=preds[:-1]
    err=pp[T]-rr[T]
    return_variance=P.var(rr)
    residual_var=P.mean(err*err)
    return "Return variance",return_variance,"residual unexplained variance",residual_var,"fraction unexplained", residual_var/return_variance

def distribution_reactive_predictions(preds,rets,data,labels):
    # select the predictions and returns that correspond to 
    # reactive control and driving forward.
    # it explains about 73% of the signal variance.
    reactind=labels.index('use_prediction')
    react=data[:,reactind]==0
    driveind=labels.index('beh_mode')
    drive=data[:,driveind]==0
    z=react * drive
    T=z[:-1]
    print(sum(T))
    rr=P.array(rets)
    pp=preds[:-1]
    #err=pp-rr
    #pp=err
    d={}
    for xx in rr[T]: 
        d[xx]=[]
    for xx,yy in zip(rr[T],pp[T]):
        d[xx].append(yy)
    pos=list(d.keys())
    vals=list(d.values())
    P.figure()
    P.boxplot(vals,positions=pos,widths=.01)
    P.xlim(-.1,1.1)
    P.xticks(P.arange(0,1.1,.1))
    P.xlabel("Ideal predictions")
    P.ylabel("Actual predictions (non-adaptive control)")

#I should also evaluate return variance and errors during reactive and
#non-reactive phases.
#plt.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
def analyze_logs(d,l,outname,innames=[]):
    #if False:
    p,np=collate_eras_collisions(d,l,3)
    print(len(p),len(np))
    plot_runs(p,np)
    P.savefig(outname+"-Runs.pdf")
    plot_overlap(p,np)
    P.savefig(outname+"-Data.pdf")

    mmp,nnp,raw=plot_means(p,np,XM=245,YM=35,YS=5,k=7999,plot_sigs=False,ylabel="Total collision time (seconds)")
    logf=open(outname+".log","w")
    logf.write("From logfiles: %s \n"%(innames))
    logf.write("number of runs: p,np = %d %d \n"%(len(p),len(np)))
    P.text(55*33.33,22*30,"Non-Adaptive Control",color="red")
    P.text(130*33.33,7*30,"Adaptive Control",color="blue")
    P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    P.savefig(outname+"-Collisions.pdf")
    logf.write("Collision Time Fraction: %s \n"%(  mmp[-1]/nnp[-1]) )
    logf.write("Collision significance: \n"+raw)



    dind=l.index("dist")
    dp,dnp=collate_eras_collisions(d,l,dind)
    mmp,nnp,raw=plot_means(dp,dnp,ylabel="Distance travelled (meters)",fixys=True,YM=100,YS=20,ymult=1000,k=7999, XM=245,plot_sigs=False)
    P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    P.text(55*33.33,50*1000,"Non-Adaptive Control",color="red")
    P.text(160*33.33,45*1000,"Adaptive Control",color="blue")
    P.savefig(outname+"-distance.pdf")
    logf.write("Distance Fraction: %s \n"%(mmp[-1]/nnp[-1]))
    logf.write("Distance significance: \n"+raw)


    out=evaluate_from_explore_onsets(d,l)
    logf.write("Prediction evaluation from probes: %s \n "%(out,))


    show_paths(d,l,0,25.1,5,RUN_START=240)
    P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    P.savefig(outname+"-0sec.pdf")
    r,p=show_paths(d,l,200,225.01,5,RUN_START=240)
    P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    P.savefig(outname+"-225sec.pdf")
    #without ideal return removal
    #show_paths(d,l,200,225.01,5,RUN_START=480)
    #P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    #P.savefig(outname+"-225sec-adaptive.pdf")
    show_paths(d,l,200,225.01,5,RUN_START=480,RHO_RETS=True)
    P.subplots_adjust(left=0.1, right=0.95, bottom=0.15, top=0.95)
    P.savefig(outname+"-225sec-adaptive-rho.pdf")
    #add prediction evaluation from non-adaptive behavior
    #  return
    # def extra():
    perf=evaluate_reactive_predictions(p,r,d,l)
    logf.write("Prediction evaluation (variance explained) from non-adative control: %s \n "%(perf,))
    print(perf)
    logf.close()
    #add distribution graph from non-adaptive behavior
    distribution_reactive_predictions(p,r,d,l)
    P.subplots_adjust(left=0.15, right=0.95, bottom=0.15, top=0.95)
    P.savefig(outname+"-non-adaptive-prediction-distribution.pdf")

def grab_lab_run():
    return grab_data_labels_nofeats(experiment_results+run013113)

def start_end(d,l):
    iter=l.index("iteration")
    start=P.find(d[:,iter])[0]
    dind=l.index("Distance")
    endd=P.find(d[:,dind]>3)[-2] 
    rind=l.index("r")
    endr=P.find(d[:,rind]>0)[-1]
    end=min(endr,endd)                              
    print(start,end)
    eraind=l.index("era")
    sind=d[start,eraind]+1
    eind=d[end,eraind]-1
    print("startera","endera",sind,eind)
    start = P.find(d[:,eraind]==sind)[0]
    end = P.find(d[:,eraind]==eind)[-1]+1 
    return start,end,(sind,eind)

def form_long_data(results):
    for x in run0131night,:#run013113, run013114, run013116, run013117:        
        print(x)
        di,li=grab_data_labels_nofeats(experiment_results+x)
        aa,bb,cc=start_end(di,li)
        results.append((di[aa:bb],cc,li))
    #I have just under 60 runs

def join_runs(list_runs):
    return P.vstack([x[0] for x in list_runs])

#a=[]
#s.form_long_data(a)
#d=d=s.join_runs(a)
#l=a[0][2]
#p,np=s.collate_eras_collisions(d,l,3)
#S.plot_runs(p,np)
#P.savefig("AllRuns.pdf")
#s.plot_overlap(p,np)
#P.savefig("AllData.pdf")
#
#mmp,nnp=s.plot_means(p,np,XM=300,YM=25,YS=2,k=7999,plot_sigs=False)
#In [713]: mmp[-1]/nnp[-1]
#Out[713]: 0.63923002651917293   = 36% less collision time
#P.savefig("All-means.pdf")
#
#dind=l.index("dist")
#dp,dnp=s.collate_eras_collisions(d,l,dind)
#s.plot_means(dp,dnp,ylabel="Distance travelled (meters)",fixys=True,YM=100,YS=20,ymult=1000,k=7999, XM=300)
#P.savefig("All-progress.pdf")

#In [763]: s.evaluate_from_explore_onsets(d,l)
#143 130
#Out[763]: (0.60940321771647066, 0.09090909090909091)
#
#s.show_paths(d,l)
#In [784]: P.xlim()
#Out[784]: (388743.26871356968, 389101.23126937513)
#P.savefig("All-predictions-reactive.pdf")


#d,l=s.check_log(s.experiment_results+s.run0131night,True)
#s.analyze_logs(d,l,"night31")

#In [108]: P.text(60*33.33,17.5*30,"Non-Adaptive Control",color="red")
#In [109]: P.text(130*33.33,7*30,"Adaptive Control",color="blue")
#In [110]: P.savefig("ar1220-labels.pdf")

#d,l=s.check_log(s.ar20,True)
#d1,l1=s.check_log(s.ar12,True)
#dd=P.vstack([d1,d])
#s.analyze_logs(dd,l,"Ar2012")
#s.analyze_logs(dd,l,"fixed-apr26",[s.fixedapr25,s.fixedapr26])
