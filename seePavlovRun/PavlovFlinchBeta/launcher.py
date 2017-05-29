import tkinter as tk
import subprocess
import socket
import multiprocessing as mp

def processPing(name):
    ret,outs = subprocess.getstatusoutput("/sbin/ping -c 1 "+ name)
    return ret==0 #0==os.system("ping -c 1 " +name)

#def pingmachine1(name):
#    ret,outs = subprocess.getstatusoutput("/sbin/ping -c 1 -W 50 "+ name)
#    return ret==0 #0==os.system("ping -c 1 " +name)
#
def pingmachine(name):
    checker=mp.Process(target=processPing, args=(name,))
    checker.start()
    checker.join(0.15)
    if checker.is_alive():
        checker.terminate()
        return False
    return checker.exitcode == 0


def netreadline(robot_name): 
    host=robot_name#"pij.local"
    port=12345
    MESSAGE=b""
    socket.setdefaulttimeout(.1)
    data=None
    print()
    try:
        print("Try",robot_name)
        sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        sock.sendto(MESSAGE, (host, port))
        print("Sent",robot_name)
        data, addr = sock.recvfrom(5000)
        print("Worked",robot_name)

    except:
        print("Failed",robot_name)
        pass
    sock.close()
    #print "received",data
    return data!=None


def startup(name):
    subprocess.call(["ipython", "--pylab","osx","viewer.py", name])


def callme(name):
    p=mp.Process(target=startup,args=(name,))
    p.start()

class Launch(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        t=MyTable(self)
        t.pack(side="top",fill="x")
        t.set(0,0,"Machine\nName")        
        t.set(0,1,"Network\nConnection")
        t.set(0,2,"Program\nRunning")
        t.set(0,3,"Launch")
        for i in range(len(t.machines)):
            t.set(i+1,0,t.machines[i])


class MyTable(tk.Frame):
    def __init__(self,parent,columns=4):
        tk.Frame.__init__(self,parent)
        self.widgets=[]
        #self.machines=["tove.cs.ualberta.ca","ras2.local","ras5.local","ras6.local"]
        self.machines=["ras2.local","ras6.local"]
        rows=len(self.machines) +1

        for row in range(rows):
            vals=[]
            for col in range(columns-1):
                txt="(%s,%s)"%(row,col)
                if col==0 and row > 1 :
                    txt=self.machines[row-1]
                label=tk.Label(self,text=txt) #width=10
                label.grid(row=row,column=col,sticky="nsew",padx=1,pady=1)
                vals.append(label)
            def localfunc():
                val=self.machines[row-1]
                return lambda : callme(val)
            if row>0:
                button=tk.Button(self,text="Launch",command=localfunc())#,state=tk.DISABLED)
                button.grid(row=row,column=columns-1)
                vals.append(button)
            else:
                label=tk.Label(self,text="0", width=10)
                label.grid(row=row,column=3,sticky="nsew",padx=1,pady=1)
                vals.append(label)
            self.widgets.append(vals)
        for col in range(columns):
            self.grid_columnconfigure(col,weight=1)
        self.after(1000, self.update_me)
        self.count=1

    def update_me(self):
        self.count+=1
        self.set(0,3,"time=% 5d"%self.count)
        for i in range(len(self.machines)):     
            name=self.machines[i]
            comm=pingmachine(name)
            self.set(i+1,1,"%s"%comm)
            alive=netreadline(name)
            self.set(i+1,2,"%s"%alive)
            if alive:
                self.widgets[i+1][3].config(state=tk.ENABLED)
            else:
                self.widgets[i+1][3].config(state=tk.DISABLED)
        #self.set(1,3,"%s"%netreadline("tove.cs.ualberta.ca"))
        self.update()
        self.after(1000, self.update_me)

    def set(self,row,col,val):
        #print(row,col,val)
        self.widgets[row][col].configure(text=val)

#out=netreadline("tove.cs.ualberta.ca")
#print(out)
#robot_name="ras6.local"
#out=netreadline2()
#print(out)
#exit(1)
if __name__=="__main__":
    app=Launch()
    app.mainloop()


