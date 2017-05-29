from __future__ import print_function
import socket
import time
#import tki_anim
import yuv
import pylab as P

policy_type_index=None

def trial():
    host="berry.local"
    port=12345
    MESSAGE="".encode("utf-8")
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    start=time.time()
    i=1;
    while True:
        now=time.time()
        sock.sendto(MESSAGE, (host, port))
        #sock.bind((host, port))
        data, addr = sock.recvfrom(5000)
        end=time.time()
        print(i,(time.time()-start)/i,end-now,len(data))
        print(data)
        print(data.decode())
        time.sleep(.001)
        i+=1

robot_name="ras6.local"
special_net_request=None
def netreadline(message=""):
    global robot_name
    host=robot_name#"pij.local"
    port=12345
    MESSAGE=message.encode("utf-8")
    socket.setdefaulttimeout(.1)
        #sock.bind((host, port))
    attempts=0
    data=None
    while attempts< 50 and data==None:
        try:
            sock = socket.socket(socket.AF_INET, # Internet
                                 socket.SOCK_DGRAM) # UDP
            sock.sendto(MESSAGE, (host, port))
            data, addr = sock.recvfrom(5000)
        except:
            if attempts< 10 or attempts%10==0:
                print("timeout: ",attempts)
            attempts +=1
        
    sock.close()
    #print "received",data
    return data.decode()

#store={}
img=None
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
def command_req(string):
    print(string)

controls=[]
class handler:
    def set_policy(self,policy):
        global controls
        if policy <0 or policy > 3:
            print("Unknown policy ",policy)
            return
        if len(controls) < 3:
            print("No controls yet","policy",policy,len(controls))
            return
        controls[policy].ax.set_axis_bgcolor('teal')
        #button.ax.figure.canvas.draw()
        for i in range(4):
            if policy != i:
                controls[i].ax.set_axis_bgcolor('white')
    def c0(self,event):
        global special_net_request
        special_net_request="policy 0"
    def c1(self,event):
        global special_net_request
        special_net_request="policy 1"
    def c2(self,event):
        global special_net_request
        special_net_request="policy 2"
    def c3(self,event):
        global special_net_request
        special_net_request="policy 3"
    def c8(self,event):
        global special_net_request
        special_net_request="remote play"
    def c9(self,event):
        global special_net_request
        special_net_request="remote pause"

button_handler=handler()

def control_buttons(fig,ax):    
    global controls
    #fig, ax = plt.subplots()
    #plt.subplots_adjust(bottom=0.2)
    freqs = P.arange(2, 20, 3)
    t = P.arange(0.0, 1.0, 0.001)
    s = P.sin(2*P.pi*freqs[0]*t)
    l, = plt.plot(t, s, lw=2)
    ax0 = plt.axes([0.0, 0.0, 0.09, 0.075])
    ax1 = plt.axes([0.1, 0.0, 0.09, 0.075])
    ax2 = plt.axes([0.2, 0.0, 0.09, 0.075])
    ax3 = plt.axes([0.3, 0.0, 0.09, 0.075])
    ax8 = plt.axes([0.8, 0.0, 0.08, 0.05])
    ax9 = plt.axes([0.9, 0.0, 0.08, 0.05])

    b0 = Button(ax0, 'only\n prediction')
    b0.on_clicked(button_handler.c0)
    b1 = Button(ax1, 'pavlovian\nmodes')
    b1.on_clicked(button_handler.c1)
    b2 = Button(ax2, 'pavlovian\nmodeless')
    b2.on_clicked(button_handler.c2)
    b3 = Button(ax3, 'actor-\ncritic')
    b3.on_clicked(button_handler.c3)
    b8 = Button(ax8, 'play')
    b8.on_clicked(button_handler.c8)
    b9 = Button(ax9, 'pause')
    b9.on_clicked(button_handler.c9)
    controls=[b0,b1,b2,b3,b8,b9,fig,ax,ax0,ax1,ax2,ax3,ax8,ax9]
    plt.draw()
    
def throw(image):
    global img
    #tki_anim.tki_anim(image)
    #t=tki_anim.tkim(image)
    #store[0]=t
    #print "faking"
    if img is None:
        f=P.figure(figsize=(9,5),dpi=130)
        ax=P.axes([0,0,1,1],frameon=False,xticks=[],yticks=[])
        img = ax.imshow(image,interpolation='none',origin='upper') #'lower'
        control_buttons(f,ax)
        f.canvas.set_window_title(robot_name)
    else:
        img.set_data(image)
    P.pause(.01)
    P.draw()

import see_log as see
import utils
import time as time_module

#("im000",extract_vals_100,see.Roffsets,.6)
fake_labels="netTime updateTime iteration r gamma obs act dist pktNum beh_mode  mode_duration vid_count prediction era learn_prediction use_prediction CliffL CliffR CliffFL CliffFR Distance Rotate IRbyte WheelBump DriveReq DriveCmd Voltage Current Overcurrent Button im000"

fake_label2="netTime updateTime iteration r gamma obs act dist pktNum beh_mode  mode_duration vid_count prediction era learn_prediction use_prediction CliffL CliffR CliffFL CliffFR Distance Rotate IRbyte WheelBump DriveReq DriveCmd Voltage Current Overcurrent image"

demonLabels="netTime updateTime iteration r gamma obs act dist pktNum action_prime  mode_duration vid_count prediction_0 prediction_1 prediction_2 era learn_prediction use_prediction CliffL CliffR CliffFL CliffFR Distance Rotate IRbyte WheelBump DriveReq DriveCmd Voltage Current Overcurrent Button image"

def reduced_image(k=100,labels=fake_label2):
    return ["image",yuv.extract_vals_500,yuv.offsets1000[:k],.505,labels]


def reproject_image(k=1000,labels=fake_label2,multiplier=16):
    return ["image",yuv.general_image_extractor(multiplier),yuv.offsets1000[:k],.5,labels]

big_image=[ x for x in yuv.large_parser]+[fake_label2]
#small_image=[ x for x in yuv.small_parser]+[fake_labels]
def large_movie(parser=big_image):#parser=yuv.large_parser,start_time=0.3,end_time=30):
    #imagelabel=-100,extractor=yuv.extract_vals_100,offsets=see.Roffsets,pavlov_baseline=.6,labels=fake_labels.split()
    global special_net_request
    imagelabel, extractor, offsets, pavlov_baseline,labels_str=parser
    labels=labels_str.split()
    imstart=labels.index(imagelabel)
    ysize,xsize=120,160
    #setup offsets
    lookup=yuv.makelookups(offsets)
    reverse_lookup=yuv.make_reversed_lookup(lookup)
    #labels=see.get_labels(logfile)
    #imstart=-100#
    #name_inds=[(n,labels.index(n)) for n in ("iteration","r","act","prediction","beh_mode")]
    #name_inds=[(n,labels.index(n)) for n in ("netTime","r","act","prediction","beh_mode","DriveCmd")]
    name_inds=[(n,labels.index(n)) for n in ("netTime","hit_decay","act","prediction_0","action_prime","DriveCmd")]
    extra_inds=[(n,labels.index(n)) for n in ["cycle","vid_count","pktNum"]]
    print(name_inds)
    #Load lines at 33ms
    rs,acts,preds,behs=[],[],[],[]
    count=0
    step=.1
    first_time=None
    now=0#start_time
    end=1000000#end_time
    #f=open(logfile)
    #utils.run_command("rm "+imageprefix+"*.jpg")
    process_i=0
    starting=time_module.time();
    oldPktNum=999
    pktCount=0
    #pktGen=0
    while True:
        process_i+=1
        if special_net_request:
            out=netreadline(special_net_request)
            print(out)
            special_net_request=None
        line=netreadline()
        #print(line)
        displayTime=(time_module.time()-starting)
        #print(process_i,"alpha",(time_module.time()-starting)/process_i)
        if line[0]=="#":
            continue
        x=line.split()#[float(x for x in line.split()]
        #print("Length",len(x))
        #if len(x)< 110:
        #    continue
        items=[(n,float(x[a])) for n,a in name_inds]
        extras=[(n,float(x[a])) for n,a in extra_inds]
        cycles=extras[0][1]
        vidCount=extras[1][1]
        pktNum=extras[2][1]
        #pktGen= pktGen+ 1 if pktNum < oldPktNum else pktGen
        #oldPktNum= pktNum
        pktCount=pktNum#pktGen*1000+pktNum
        bumped_recently = int(items[1][1] > .85 )
        rs=[bumped_recently]+rs[:99]
        acts=[items[-1][1]*.25]+acts[:99]
        preds=[items[3][1]]+preds[:99]
        behs=[yuv.ite(items[4][1]==3, 1 , 0)  ]+behs[:99]
        vals=extractor(x,imstart)
        #print ("phase 1", vals.shape)
        if first_time==None:
            #if items[4][1] <0:  #initial pause time
            #    continue
            first_time=items[0][1]
            first_time=0
            print(first_time)
            continue
        #print "phase 2"
        time=items[0][1]-first_time
        #print("pre",items[0][1],first_time,time)
        if policy_type_index:
            #print("Policy type",x[policy_type_index])
            button_handler.set_policy(int(x[policy_type_index]))
        #if time > end:
        #    break
        #while now==0 or time > now:
        #print "phase 3"
        #print("post")
        
        if True:

            now+=step
            #yuvimg=yuv.fill_offset_vals(vals,lookup,(ysize,xsize))
            yuvimg=yuv.fill_offset_vals_reversed(vals,reverse_lookup,(ysize,xsize))
            #print "im len", len(yuvimg)
            rgb=yuv.rgbFromyuv422Fast(yuvimg)
            #rgb=P.zeros((240,320,3),P.uint8)
            data=[]
            data.append(acts)
            bb=acts[0]*4
            name=yuv.ite(bb==0, "(Drive)", yuv.ite(bb==4, "(Stop)",yuv.ite(bb==3,"(Reverse)","(Turn)")))
            data.append("motors=%d %s"%(bb,name))
            data.append(rs)
            name=yuv.ite(rs[0]==1, "(Bump)", "(No Bump)")
            data.append("bumper =%d %s"%(int(rs[0]),name))
            if pavlov_baseline:
                data.append([[pavlov_baseline]]+preds)            
            else:
                data.append(preds)
            #print "phase 4"
            data.append("prediction=%4.3f"%preds[0])
            #data.append(behs)
            #data.append("testing prediction=%d"%behs[0])
            data.append("")
            data.append("")
            data.append(robot_name)
            data.append("")
            data.append("xTD: %3.1fms viz: %4.1fms"%((1000*time/cycles),1000*(displayTime/process_i)))
            data.append("cam: %3.1fms bot: %3.1fms"%((1000*time/vidCount),1000*time/pktNum))
            data.append("")
            data.append("time=%4.2f seconds"%time)
            image=image_annotate(rgb,data,showPred=preds[0],isBump=int(1==rs[0]),showAct=acts[0]);
            #return image
        throw(image)
            #count+=1
    #f.close()
    #ffmpeg the results
    #utils.run_command("ffmpeg  -y -r 10 -i "+imageprefix+"%06d.jpg  -sameq "+ moviename)
#yuv.large_movie_from_logfile("Waller-y2014-m01-d27-h17-m02-s41.logfile","/tmp/imx","/tmp/visualLearningSmall.mp4",parser=yuv.small_parser,start_time=20,end_time=300)

def image_annotate(rgb,textlines,isBump=None, showPred=None,showAct=None):
    import PIL
    import PIL.Image
    import PIL.ImageDraw
    import PIL.ImageFont

    #font=PIL.ImageFont.truetype("/usr/X11/lib/X11/fonts/TTF/Vera.ttf", 18)
    font=PIL.ImageFont.truetype("/Library/Fonts/Arial.ttf", 18)
    im1=PIL.Image.fromarray(rgb,"RGB")
    imoff=240
    off=0
    vz=40
    im2=im1.resize((640,480))
    im=PIL.Image.new("RGB",(640+240,480))
    im.paste(im2,(240,0))
    #print im.size
    id=PIL.ImageDraw.ImageDraw(im)
    kk=5
    baroff=imoff-kk
    if isBump!=None:
        if isBump:
            id.line([(imoff,kk),(imoff+639,kk),(imoff+639-kk,480-kk),(imoff+kk,480-kk),(imoff+kk,kk)],fill="red",width=2*kk)
    if showPred!=None:
        y=480*(1-showPred)
        id.line([(baroff,480),(baroff,y)],fill="yellow",width=2*kk)
        id.line([(baroff,y),(baroff,0)],fill="blue",width=2*kk)
    if showAct!=None:
        #replace these with an a hollow arrow in the center of the image.
        cx,cy = imoff+320,240
        a1=-10,10
        a2=10,10
        a3=10,-10
        a4=-10,-10
        p=0,-30
        if showAct==0:
            id.line([(x+cx,y+cy) for x,y in [a1,a2,p,a1]],width=2,fill="white")
        elif showAct==.25:
            id.line([(y+cx,x+cy) for x,y in [a1,a2,p,a1]],width=2,fill="white")
        elif showAct==.75:
            id.line([(x+cx,-y+cy) for x,y in [a1,a2,p,a1]],width=2,fill="white")
        else:
            id.line([(y+cx,x+cy) for x,y in [a1,a2,a3,a4,a1]],width=2,fill="white")
        #y=480*showAct
        #id.line([(3*kk,480-kk),(3*kk,y)],fill="green",width=2*kk)
        #id.line([(3*kk,y),(3*kk,kk)],fill="purple",width=2*kk)
    count=1
    #im.show()
    for item in textlines:
        #print item,type(item)
        #print item,type(item)
        if type(item)==type(""):
            id.text((off+10,count*vz),item,font=font)
            count-=.5
        elif type(item)==type([]):
            if len(item) > 0 and type(item[0])==type([]):
                q=item[0][0]
                vals=[(off+10+200,-q*vz + (count+1)*vz),(off+10,-q*vz + (count+1)*vz)]
                id.line(vals,fill="#4080ff",width=1)
                item=item[1:]
            z=P.array(item)
            n=P.clip(z,-1,1)
            vals=[(off+10+200-2*i,-n[i]*vz + (count+1)*vz)  for i in range(len(item))]
            #vals=[(vals[0][0], (count+1.25)*vz )]+vals+[(vals[-1][0],(count+1.25)*vz )]
            id.line(vals,fill="green",width=2)
            count+=.1
        else:
            id.text((off+10,count*vz),item[0],fill=item[1])
        count+=1
    del id
    #im.show()
    #im.save(savename)
    return im

#large_movie()

#I should add in multiple prediction support, but the code should be
#cleaned a bit first.
import sys
#control_buttons()
if __name__ == "__main__":
   args=sys.argv[1:]
   if len(args) >0:
       robot_name=args[0]
   print("Connecting to robot:",robot_name)
   data=netreadline("info") #This does nothing useful for the old code.
   #print(data)
   lines=data.split("\n")
   constants=[x for x in lines if len(x)> 10 and x[1]=="G"]
   print("Constants")
   multiplier=8
   if constants !=[]:
       key_vals=[x.split("=") for x in constants[0].split()]
       for kv in key_vals:
           if len(kv) > 1:
               print(kv[0],kv[1])
               if kv[0]=="byte_rshift":
                   multiplier = 1 << int(kv[1])
       print("\n")

   labels=[x for x in lines if len(x)> 10 and x[1]=="L"][0]
   policy_type_index=labels.split().index("policy_type") -1
   data=netreadline().split()
   n=len(data)
   a=len(data[-1])
   print("image featurization (bytes,tile_width):",a,multiplier)
   #if a > 500:
   #large_movie(reproject_image(a,multiplier=multiplier))
   #Now only works with the new code as it needs labels.  Revert to above line on failure.
   newlabels=" ".join(labels.split()[1:])
   print(newlabels)
   large_movie(reproject_image(a,multiplier=multiplier,labels=newlabels))
   #else:
   #    large_movie(reduced_image(a))
  
