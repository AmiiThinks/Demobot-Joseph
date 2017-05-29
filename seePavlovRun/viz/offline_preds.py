import pylab as P

fname="/Users/jmodayil/data/pavlov/Waller-y2014-m01-d31-h18-m31-s19.logfile"
def getdata(fname=fname):
    q=P.loadtxt(fname)
    #q.shape
    #Out[3]: (377299, 129)
    return q

def getlabels(fname=fname):
    f=open(fname,"r")
    f.readline();f.readline();f.readline();
    l=f.readline()
    labels=l.split()[1:]
    #last 100 are the image features.
    return labels

def just_imfeats(q):
    qs=q[:,-100:]
    return qs

import time
def make_cov_imfeats(qs): #too slow with python.
    m=16
    n=100
    mn=m*n
    if qs.shape[1]!= n:
        print "Unexpected shape - examine the code: "
        return
    counts=P.zeros(mn)
    cov=P.zeros((mn,mn))
    norm=1./len(qs)
    ln=len(qs)
    start=time.time()
    for k in range(ln):
        line= qs[k]
        if k%100==0:
            print "%d of %d, %f"%(k,ln,time.time()-start)
        active=[m*i+line[i]  for i in range(n)]
        #for x in active:
        #counts[active]+=1
        for i in active:
            for j in active:
                cov[i,j]+=1#inds=[(i,j) for i in active for j in active]
        #cov[inds]+=1
    cov*= norm
    #counts *=norm
    return cov
#
def run_command(s,want_output=False):
    import os
    a= os.popen(s).readlines()
    if want_output:
        return a
    return ""

def make_cov_imfeats_file(filename=fname): 
    run_command("make cov")
    run_command("./cov "+filename+" inds.in")

def eigen_cov():
    cov=P.loadtxt("/tmp/cov.out")
    eval,evec=P.eigh(cov)
    return eval,evec

def compute_covinv(eval,evec,cutoff=.000001):
    eval=[P.fabs(x) for x in eval]
    #each eigenvector gets scaled by 1/sqrt(lambda)
    for i in range(len(evec)):
        if eval[i] > cutoff:
            delta=P.sqrt(1/eval[i])
        else:
            delta=0
        evec[:,i]=delta*evec[:,i]
    inv=P.dot(evec,evec.T)
    return inv

def compute_lstsq(ATb,cov_inv):
    #if Ax=b then x~=(A^tA)^-1 A^t b  via pseudo inverse forms
    #and cov=A^t A
    #so x~= cov_inv A^t b 
    #vals=P.dot(A.T,b)
    out=P.dot(cov_inv,ATb)
    return out

def get_inds_return(l,d):
    bind=l.index("beh_mode") #index 9
    pind=l.index('use_prediction') # index 15
    rind=l.index('r') #index 3
    offs=(d[:,pind]==0) * (d[:,bind]==0)
    Ret=0
    rets=[]
    for i in range(len(offs)-2,-1,-1):        
        if offs[i] and d[i+1,rind]==1:
            Ret=1
            rets.append((i,Ret))
        elif offs[i]:
            Ret*=.95
            rets.append((i,Ret))
        else:
            Ret=0
    return P.array(rets).T
    
def make_Ab(inds_rets,d):
    i=[int(x) for x in inds_rets[0]]
    A=d[i,29:]
    b=inds_rets[1]
    outs=[]
    for j in range(100):
        for k in range(16):
            #ind=k+j*16
            val=P.sum(b[A[:,j]==k])
            print j,val
            outs.append(val)
    return P.array(outs)

def make_preds(d,w):
    preds=[]
    for v in d:        
        p=P.sum(w[16*i+v[i+29]]  for i in range(100))
        preds.append(p)
    return preds

def write_inds(inds_in,indsfile="inds.in"):
    f=open(indsfile,"w")
    inds=inds_in[:]
    inds.sort()
    for x in inds:
        f.write("%d\n"%int(x))
    f.close()
#want pind of 0 and bind of 0
#compute returns

#Find the indices on which to restrict true returns - on-policy and
#driving forward.

#o.comparem(ir[0],ir[1],d[:,12],preds)
def comparem(inds,rets,online,offline):
    ons=P.array([ online[int(i)]-r for i,r in zip(inds,rets)])
    offs=P.array([ offline[int(i)]-r for i,r in zip(inds,rets)])
    print P.sqrt(P.mean(ons*ons))
    print P.sqrt(P.mean(offs*offs))
#In [134]: o.comparem(ir[0],ir[1],d[:,12],preds)
#0.326360284004
#0.336679145411

"""
d=o.getdata(s.ar20)
l=o.getlabels(s.ar20)

ir=o.get_inds_return(l,d)
atb=o.make_Ab(ir,d)
o.make_cov_imfeats_file(s.ar20)
In [114]: eval,evec=o.eigen_cov()
In [116]: ci=o.compute_covinv(eval,evec)
In [117]: w=P.dot(ci,atb)
In [118]: preds=o.make_preds(d,w)
Pind=l.index('prediction')# 12
pind=l.get_index("prediction")
plot returns, predictions, and pind=lstsqaures
o.comparem(ir[0],ir[1],d[:,12],preds)
"""

#I expect the result i want will only hold for the end of the dataset, which involves some additional coding.
