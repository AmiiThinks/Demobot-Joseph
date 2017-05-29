import pylab as P
class GTD:
    def __init__(self,n,alpha,elambda = 0,alpha2multiplier=.01):
        self.n=n
        self.w=P.zeros(n)
        self.vv=P.zeros(n)
        self.e=P.zeros(n)
        self.alpha=alpha
        self.alpha2=alpha2multiplier*self.alpha
        self.elambda=elambda
        self.delta=0
        #print self.alpha, self.n, self.gamma,self.elambda, self.e.shape
    def update(self,rho_x,x,r,xn,gamma_xn):
        #print "rho",rho_x,"r",r,"x",x.shape,"xp",xp.shape
        delta=r + gamma_xn *P.dot(self.w,xn) - P.dot(self.w,x)
        self.e= rho_x*(x + self.e)
        #print P.norm(self.e),P.norm(self.w)
        self.w += self.alpha*( delta*self.e - gamma_xn*(1-self.elambda)*P.dot(self.e, self.vv)* xn)
        self.vv +=self.alpha2 *(delta * self.e - P.dot(x,self.vv)*xn)
        self.e*=gamma_xn*self.elambda
        self.delta=delta  
    def predict(self,x):
        return P.dot(self.w,x)


#returns only make sense for 0-1 recognizers (not standard rhos).
#I still want to make them. 
def compute_returns(rs,gammas,recognizers_prevstep):
    rets=[]
    ret=0
    ret_mass=0
    for i in range(len(rs)-1,-1,-1):
        ret=rs[i]+gammas[i]*ret
        ret_mass = recognizers_prevstep[i]*((1-gammas[i]) + (gammas[i]*ret_mass))
        rets.append((ret,i,ret_mass))
    rets.reverse()
    #print "Returns:",len(rs),len(rets)
    return rets

def restrict_returns(ret_i_wt, min_wt=.95):
    out=[]
    for r,i,wt in ret_i_wt:
        if wt > min_wt:
            out.append((i,r))
        else:
            out.append((i,P.nan))
    return out

def compute_predictions(rs,gammas,rhos,xs):
    X=xs[0]
    n=len(X)
    T=len(xs)
    m=sum(X)
    print "M",m
    predictor=GTD(n=n,alpha=0.1/m,elambda=0.95)
    predictions=[predictor.predict(xs[0])]
    for i in range(1,T):
        p=predictor.predict(xs[i])
        predictions.append(p)
        predictor.update(rhos[i-1],xs[i-1],rs[i],xs[i],gammas[i])
        #print rhos[i-1],rs[i],gammas[i],p, #P.nonzero(xs[i-1])
    return predictions


def compute_predictions_f(data,r_f,gamma_f,rho_f,x_f):
    x=x_f(data[0])
    n=len(x)
    T=len(data)
    m=sum(x)
    predictor=GTD(n=n,alpha=0.1/m)
    xl=x_f(data[0])
    predictions=[predictor.predict(xl)]
    for i in range(1,T):
        xn=x_f(data[i])
        p=predictor.predict()
        predictions.append(p)
        predictor.update(rho_f(data[i-1]),xl,r_f(data[i]),xn,gamma_f(data[i]))
        xl=xn[:]
    return predictions
        
