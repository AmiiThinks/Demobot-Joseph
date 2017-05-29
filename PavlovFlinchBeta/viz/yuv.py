#process yuv data in a sane way

import pylab as P
import see_log as see
#All local nao yuv are really just y-channel
ysample="/Users/jmodayil/projects/NAO/trials/trial2-vid00093.yuv"
y2="/Users/jmodayil/data/naoall/gamma-vid21310.yuv"
y3="/Users/jmodayil/projects/NAO/trials/trial1-vid00143.yuv"

#a fake ppm
yuvsample="/Users/jmodayil/Data/000105.ppm"
yuv2="/Users/jmodayil/Dropbox/raspi/video/image.yuvs"
def getydata(filename,skipbytes=12,size=(120,160)):
    raw=P.frombuffer(open(filename).read()[skipbytes:],P.uint8)
    print(len(raw))
    raw.shape=size
    yimg=raw
    return yimg

def getyuv422data(filename,skipbytes=15,size=None):
    raw=P.frombuffer(open(filename).read()[skipbytes:],P.uint8)
    #4*3*2
    print(len(raw))
    if size==None:
        scale=int(P.sqrt(len(raw)//(4*3*2)))
        size=scale*3,scale*4,2
    raw.shape=size
    yimg=raw
    return yimg


def rgbpix(c,d,e):
        r=P.clip((298*c         + 409*e + 128)// 256,0, 255)
        g= P.clip((298*c -100*d - 208*e + 128)// 256,0, 255)
        b= P.clip((298*c + 516*d + 128)// 256,0, 255)
        return r,g,b

def rgbFromyuv422(yuvdata):
    y,x,b=yuvdata.shape
    out=P.zeros((y,x,3),P.uint8)
    print(out.shape,y,x)

    def rgb(c,d,e):
        r=P.clip((298*c         + 409*e + 128)// 256,0, 255)
        g= P.clip((298*c -100*d - 208*e + 128)// 256,0, 255)
        b= P.clip((298*c + 516*d + 128)// 256,0, 255)
        return r,g,b
    for yy in range(y):
        for xx in range(0,x,2):
            y1,u=yuvdata[yy,xx]
            y2,v=yuvdata[yy,xx+1]
            c1=y1-16
            c2=y2-16
            d=u-128
            e=v-128
            out[yy,xx]=rgb(c1,d,e)
            out[yy,xx+1]=rgb(c2,d,e)
    rgb=out
    print("done")
    return rgb



def rgbFromyuv422Fast(yuvdata):
    #slow=rgbFromyuv422(yuvdata)
    y,x,b=yuvdata.shape
    #temp=P.zeros((y,x,3),P.float32)
    rgb=P.zeros((y,x,3),P.uint8)
    #print out.shape,y,x

    #def rgb(c,d,e):
    #    r=P.clip((298*c + 409*e + 128)/ 256,0, 255)
    #    g= P.clip((298*c -100*d - 208*e + 128)/ 256,0, 255)
    #    b= P.clip((298*c + 516*d + 128)/ 256,0, 255)
    #    return r,g,b
    yuvflat=P.array(yuvdata,dtype=P.int32)
    yuvflat=yuvflat.reshape((y*x*2))
    y1s=yuvflat[0::4]-16
    y2s=yuvflat[2::4]-16
    us=yuvflat[1::4]-128
    vs=yuvflat[3::4]-128
    yuvs=P.empty((y*x,3),P.float32)
    #print y*x,yuvs.shape,y1s.shape,us.shape,yuvflat.shape
    yuvs[::2,0]=y1s
    yuvs[1::2,0]=y2s
    yuvs[::2,1]=us
    yuvs[1::2,1]=us
    yuvs[::2,2]=vs
    yuvs[1::2,2]=vs
    #print yuvs
    Trans=P.array([[298,0,409],[298,-100, -208],[298,516,0]]).T
    temp=P.dot(yuvs,Trans)
    #print temp[1],yuvs[1],(yuvdata[0,0],yuvdata[0,1]),rgbpix(yuvs[1][0],yuvs[1][1],yuvs[1][2])
    temp+=128
    temp//=256
    #print temp[1]
    #print temp.shape
    temp=temp.reshape((y,x,3))
    P.clip(temp,0,255,rgb)
    #print slow[0,1]
    #print rgb[0,1]
    #sfd
    #    print "done1"
    return rgb

def jpgfromrawyuvfile(rawyuvfilename):
    yuvimg=getyuv422data(rawyuvfilename,skipbytes=0);
    rgb=rgbFromyuv422(yuvimg)
    P.imshow(rgb)
    P.savefig(rawyuvfilename+".pdf")
def plotydata(yimg):
    P.imshow(yimg,P.cm.Greys_r)

def plotyuvdata(yuvimg):
    rgb=rgbFromyuv422(yuvimg)
    P.imshow(rgb)

def plotrgbdata(rgbimg):
    P.imshow(rgbimg)

def plotyuvfile(filename):
    d=getyuv422data(filename,skipbytes=0,size=(120,160,2))
    plotyuvdata(d)
    
def makeoffsets(number,size):
    import random
    return random.sample(list(range(number)),size)

def loadoffsets(filename):
    return P.loadtxt(filename)

def loadyuvoffset(off_vals_file):
    return P.loadtxt(off_vals_file)

def maskpixels(yuvdata,mask=128+64+32):
    return yuvdata &mask

import heapq
def makelookuptable(offset_locs,dist2,ysize,xsize):    
    #make an array of things to fill, initialized with dist,pix,kname,kpix
    #make a queue of neighbors, initialized with offset,0
    #add neighbors to queue, ensuring they are valid
    table=dict([ ((i,j),(-1,0,P.inf)) for i in range(ysize) for j in range(xsize)])
    queue=[]
    for kname,kpix in offset_locs:
        heapq.heappush(queue,(0,kpix, kname, kpix))
    #print heapq.heappop(queue)
    while len(queue) >0:
        d,yx,kname,kpix=heapq.heappop(queue)
        y,x=yx
        if table[y,x][0]==-1 or table[y,x][2] > dist2(yx,kpix):
            table[y,x]=(kname,kpix,dist2(yx,kpix))
            for i,j in [(-1,0),(1,0),(0,1),(0,-1)]:
                b,a=y+j,x+i
                if a<0 or b<0 or a>=xsize or b>=ysize or (i==0 and j==0):
                    continue
                if table[b,a][2] > table[y,x][2]:  
                    heapq.heappush(queue,(table[y,x][2],(b,a),kname,kpix) )
    return table

def makelookups(offsets,size=(120,160)):
    ysize,xsize=size
    sqr =lambda x: x*x
    dy = lambda yx,ab : sqr(yx[0]-ab[0]) + sqr(yx[1] - ab[1])
    #duv = lambda yx,ab : sqr(yx[0]-ab[0]) + 4*sqr(yx[1] - ab[1])

    row=2*xsize
    yloc=lambda k: (k//row,(k//2) %xsize)
    #uvloc=lambda k: (k/row,(k/4)%(xsize/2))

    ys,us,vs=[],[],[]
    for i in range(len(offsets)):
        k=offsets[i]
        if k%2==0:
            ys.append((i,yloc(k)))
        elif k %4 ==1:
            us.append((i,yloc(k)))
        elif k %4 ==3:
            vs.append((i,yloc(k)))
    #ys=[(k,yloc(k)) for k in offsets if k %2 ==0]
    #us=[(k,uvloc(k)) for k in offsets if k %4 ==1]
    #vs=[(k,uvloc(k)) for k in offsets if k %4 ==3]

    ytable=makelookuptable(ys,dy,ysize,xsize)
    print("ys",xsize,ysize)
    utable=makelookuptable(us,dy,ysize,xsize)
    vtable=makelookuptable(vs,dy,ysize,xsize)
    netlookuptable=dict()#P.zeros((ysize,xsize,2),dtype=P.int32)
    for i in range(ysize):
        for j in range(0,xsize,2):
            netlookuptable[(i,j,0)]=ytable[i,j][0]
            netlookuptable[(i,j,1)]=utable[i,j][0]
            netlookuptable[(i,j+1,0)]=ytable[i,j+1][0]
            netlookuptable[(i,j+1,1)]=vtable[i,j+1][0]
    return netlookuptable

def fill_offset_vals(vals,lookup,size=(120,160)):
    yuvimage=P.zeros((size[0],size[1],2),P.uint8)
    #fill black
    for key,r in list(lookup.items()):
        yuvimage[key]=vals[r]
    return yuvimage

def make_reversed_lookup(lookup):
    top=max(lookup.values())+1
    R=[[] for i in range(top)]
    for k,v in list(lookup.items()):
        R[v].append(k)
    for i in range(len(R)):
        R[i].sort()
        z=P.array(R[i]).T
        R[i]=(z[0],z[1],z[2])
    return R

def fill_offset_vals_reversed(vals,R,size=(120,160)):
    yuvimage=P.zeros((size[0],size[1],2),P.uint8)
    #fill black
    for i in range(len(R)):
        yuvimage[R[i][0],R[i][1],R[i][2]]=vals[i]
    return yuvimage

def addrgb_offet_points(rgb,offsets):
    for p in offsets:
        q=p%4
        x=(p//2) %160
        y=p//(2*160)
        r,g,b,=0,0,0
        if q==0 or q==2:
            r=255
        if q==1:
            g=255
        if q==3:
            b=255
        rgb[y,x, 0]=r
        rgb[y,x, 1]=g
        rgb[y,x, 2]=b

def get_offset_projected_image(image,offsets,lookup,mask=255,addpoints=False):
    shape=image.shape
    ysize,xsize=image.shape[0:2]
    image.shape=(ysize*xsize*2,)
    vals=[image[o]&mask for o in offsets]
    image.shape=shape
    yuvimg=fill_offset_vals(vals,lookup,(ysize,xsize))
    #plotyuvdata(yuvimg)
    rgb=rgbFromyuv422(yuvimg)
    if addpoints:
        addrgb_offet_points(rgb,offsets)
    return rgb

def plot_offset_projected_image(image,offsets,lookup,mask=255):
    rgb=get_offset_projected_image(image,offsets,lookup,mask)
    plotrgbdata(rgb)
    return rgb

def rgbsave(rgb,name):
    import PIL.Image
    f=PIL.Image.fromarray(rgb)
    f.save(name)

#f="/Users/jmodayil/Dropbox/PavlovOffPolicy/snapshot-1391192384-46066.yuv"
#d=yuv.getyuv422data(f,skipbytes=0)
#yuv.examplify(d,"92384")

def examplify(yuv,basename):
  #Take yuvdata, add the offset dots, and show the nearest-pixel reconstruction
    import see_log as see
    offsets=see.Roffsets
    examplify_offsets(yuv,basename,offsets)

def examplify_offsets(yuv,basename,offsets,mask=128+64+32+16,addpoints=False):
    r=rgbFromyuv422Fast(yuv)
    addrgb_offet_points(r,offsets)
    rgbsave(r,basename+".png")
    lookup=makelookups(offsets)
    out=get_offset_projected_image(yuv,offsets,lookup,mask=mask,addpoints=addpoints)
    rgbsave(out,basename+"-recon.png")

def plot_offset_image(yuvimage,count):
    y,x,b=yuvimage.shape
    size=y*x*b
    off=makeoffsets(size,count)
    lookup=makelookups(off,(y,x))
    return plot_offset_projected_image(yuvimage,off,lookup)
        

def image_annotate_save(rgb,textlines,savename,isBump=None, showPred=None,showAct=None):
    import PIL
    import PIL.Image
    import PIL.ImageDraw
    import PIL.ImageFont

    font=PIL.ImageFont.truetype("/usr/X11/lib/X11/fonts/TTF/Vera.ttf", 18)
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
        if type(item)==type(""):
            id.text((off+10,count*vz),item,font=font)
            count+=.2
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
    im.save(savename)

#make_movie_from_logfile("Waller-y2014-m01-d27-h17-m02-s41.logfile","/tmp/imx","/tmp/movie.mp4")

def process_line_std(line,name_inds):
    vals=line.split()
    

# def make_movie_from_logfile(logfile,imageprefix,moviename,offsets=None):
#     import see_log as see
#     import utils
#     if offsets==None:
#         offsets=see.Roffsets
#     ysize,xsize=120,160
#     #setup offsets
#     lookup=makelookups(offsets);
#     labels=see.get_labels(logfile)
#     imstart=labels.index("im000")
#     name_inds=[(n,labels.index(n)) for n in ("iteration","r","act","prediction","beh_mode")]
#     #Load lines at 33ms
#     rs=[]
#     acts=[]
#     preds=[]
#     behs=[]
#     count=0
#     step=.1
#     now=30
#     f=open(logfile)
#     for line in f.readlines():
#         if line[0]=="#":
#             continue
#         x=[float(x) for x in line.split()]
#         items=[(n,x[a]) for n,a in name_inds]
#         rs=[items[1][1]]+rs[:99]
#         acts=[items[2][1]]+acts[:99]
#         preds=[items[3][1]]+preds[:99]
#         behs=[ite(items[4][1]==4, 1 , 0)  ]+behs[:99]
#         vals=P.array(x[imstart:])*16
#         time=items[0][1]*.03
#         if time > 300:
#             break
#         print now,time,count
#         while now==0 or time > now:
#             now+=step
#             yuvimg=fill_offset_vals(vals,lookup,(ysize,xsize))
#             rgb=rgbFromyuv422Fast(yuvimg)
#             data=[]
#             data.append("time=%4.3f"%time)
#             data.append(rs)
#             data.append("bumper=%d"%rs[0])
#             data.append(acts)
#             data.append("action=%d"%acts[0])
#             data.append(preds)
#             data.append("prediction=%4.3f"%preds[0])
#             data.append(behs)
#             data.append("behaviour mode=%d"%(behs[0]*4))
#             #data=[("%s=%4.3f"%(n,v),(255,255,255)) for n,v in items]
#             image_annotate_save(rgb,data,imageprefix+"%06d.jpg"%count);
#             count+=1
#     f.close()
#     #ffmpeg the results
#     utils.run_command("ffmpeg -r 10 -i "+imageprefix+"%06d.jpg "+ moviename)

def ite(a,b,c):
    if a:
        return b
    return c

offsets1000=[4396, 7882, 6214, 13401, 28419, 31917, 32134, 18636, 35215, 32733, 17001, 28236, 37547, 29600, 10230, 17256, 13026, 28199, 19887, 16305, 5131, 34770, 31807, 34762, 994, 5756, 15158, 34807, 25817, 35689, 18061, 753, 4491, 20295, 16044, 4494, 9172, 21702, 29955, 16312, 9451, 34135, 26715, 12689, 6049, 11524, 14897, 10725, 7943, 29762, 36939, 6465, 28875, 1671, 37430, 36707, 1110, 5932, 32154, 27069, 3776, 2489, 87, 22306, 10756, 27424, 9250, 30892, 35971, 27194, 255, 31169, 35891, 24148, 16925, 22291, 7420, 16760, 181, 26446, 13148, 34399, 9438, 26849, 11387, 4548, 9239, 13812, 13278, 25713, 14639, 5250, 33887, 22156, 22649, 21075, 23485, 14706, 4704, 6309, 30766, 13037, 11146, 16404, 14060, 29567, 1384, 11647, 30734, 18308, 14824, 23641, 27735, 3249, 1177, 14485, 19151, 35519, 25088, 30981, 2461, 19614, 1039, 15873, 5025, 1013, 12441, 14343, 28887, 4986, 31221, 33584, 23976, 17775, 13964, 14560, 30169, 31913, 26700, 7195, 2697, 37297, 13316, 18757, 37231, 26663, 30813, 29107, 21802, 31727, 6146, 38151, 37761, 25041, 7224, 20403, 26998, 29900, 22255, 29592, 32319, 9194, 28069, 3966, 27491, 9379, 35652, 22584, 12765, 17029, 7536, 34966, 19301, 10062, 10328, 27879, 10015, 20860, 12057, 19434, 22527, 24614, 17937, 5556, 36450, 13438, 2706, 30320, 28245, 24743, 27617, 15648, 5992, 19400, 35174, 33711, 35991, 34072, 20575, 31741, 3802, 20710, 20986, 19109, 1596, 25573, 26511, 16242, 24453, 11205, 24915, 8455, 18781, 23112, 5898, 4877, 26103, 18547, 6862, 21186, 4507, 23079, 3221, 12754, 19762, 24923, 10987, 23907, 31034, 9378, 19392, 37894, 34264, 29556, 1738, 29844, 8239, 25246, 900, 25389, 38084, 34261, 13726, 38059, 5559, 26304, 36274, 16534, 30226, 37288, 35524, 376, 12081, 10887, 35406, 35718, 23750, 20004, 34760, 10601, 28366, 9849, 32307, 9201, 19042, 17607, 596, 30690, 14447, 14999, 7756, 27033, 24238, 7806, 20215, 4314, 34895, 20278, 35401, 9673, 21447, 33864, 22681, 24034, 2386, 33932, 38345, 26296, 33560, 2023, 20475, 5904, 30999, 32504, 38142, 4634, 35926, 3007, 4343, 16626, 38393, 2805, 15398, 1524, 27218, 13994, 23790, 22107, 5921, 19982, 25603, 2645, 34802, 24319, 18747, 34590, 9217, 9401, 7022, 32726, 27993, 31498, 33980, 28880, 30707, 20817, 21480, 35209, 8160, 10507, 23163, 17330, 33686, 15332, 7131, 3912, 2596, 28116, 16753, 1086, 30623, 803, 21417, 35614, 15789, 4961, 2765, 26496, 24766, 6888, 733, 13878, 18325, 9730, 23889, 25333, 2505, 30038, 24993, 35649, 26881, 30672, 6381, 9168, 18963, 21540, 16461, 702, 11520, 36926, 2631, 6079, 37763, 5600, 11778, 31895, 12512, 12960, 26842, 38277, 36502, 36299, 33994, 34042, 31448, 17723, 38238, 5211, 36599, 11266, 21977, 33033, 31577, 14848, 26538, 31738, 4990, 10900, 38108, 16886, 25417, 34594, 31135, 5104, 23881, 28449, 30948, 36421, 22537, 37964, 31761, 35618, 27856, 10950, 3898, 36565, 11079, 37190, 18504, 2036, 2421, 18818, 24767, 3878, 17731, 31887, 4847, 36774, 34029, 16518, 15701, 2015, 17263, 22283, 19733, 11109, 12759, 23236, 5918, 17580, 31131, 2217, 2014, 19587, 24891, 21579, 30026, 19944, 20157, 21406, 36297, 6669, 5701, 20715, 24859, 20031, 8286, 11686, 29555, 27951, 29050, 33070, 33185, 36898, 30146, 29755, 31531, 21366, 26883, 26185, 2453, 4828, 27517, 20205, 14219, 22308, 19554, 15780, 21274, 10157, 38220, 21994, 12320, 10742, 1, 37023, 13068, 22799, 9937, 18397, 28375, 4157, 12593, 15791, 38016, 2707, 9519, 10443, 23312, 5273, 6045, 6187, 14531, 17269, 35481, 15236, 27798, 867, 28943, 28712, 23031, 81, 15171, 8195, 11096, 32634, 17587, 8750, 25932, 4807, 19789, 10374, 16703, 26943, 11770, 2139, 20147, 17412, 2998, 7062, 17072, 620, 19220, 23254, 9292, 23258, 3507, 16122, 14970, 4262, 12033, 31532, 16600, 29381, 5817, 23186, 25149, 35467, 34673, 903, 17811, 36294, 20311, 2246, 19002, 24208, 21306, 20088, 21263, 18202, 3405, 29741, 3698, 17841, 17927, 31679, 12007, 4637, 25384, 28339, 7954, 23405, 29746, 17689, 24896, 21899, 34805, 35080, 20646, 29899, 19433, 24507, 27747, 35046, 21764, 3101, 2978, 26864, 33803, 5942, 28892, 32836, 36484, 19746, 12144, 32449, 30900, 37975, 7351, 37506, 8222, 1300, 29667, 32001, 18771, 34251, 35360, 5809, 3673, 37659, 37228, 3397, 29252, 33024, 17918, 26934, 8637, 25361, 799, 1915, 8763, 22739, 33269, 35466, 18262, 20435, 950, 34840, 21861, 1744, 32507, 25562, 12171, 15578, 16912, 38010, 13654, 14986, 31430, 18395, 4098, 36127, 38333, 20894, 7089, 34879, 1910, 35605, 12890, 31706, 29763, 38172, 24494, 30271, 7968, 17174, 21011, 1122, 35331, 16173, 21608, 12672, 29367, 163, 7030, 12967, 11905, 8624, 26344, 34114, 16679, 18762, 36995, 29976, 12839, 36347, 30373, 7773, 7302, 12686, 34814, 29356, 20606, 30292, 19773, 20988, 10027, 16420, 7972, 13720, 31566, 3292, 31188, 30722, 19867, 35803, 17842, 11662, 34107, 7596, 3506, 37665, 10709, 237, 36511, 9419, 37273, 25387, 12552, 37034, 37969, 23697, 6871, 14596, 23393, 1870, 12317, 14115, 2226, 7629, 8272, 8665, 19844, 32816, 4127, 7804, 37632, 25636, 27726, 8450, 16896, 31877, 15174, 10313, 27645, 36568, 9731, 15383, 10819, 30906, 11802, 28715, 10318, 14205, 21719, 7649, 14495, 36232, 10000, 28697, 31686, 25998, 7477, 17215, 36135, 34724, 3133, 6460, 31747, 12231, 8742, 4282, 14833, 8965, 3289, 17542, 5800, 26347, 19775, 28725, 1765, 29059, 14828, 12264, 7202, 36893, 25270, 30089, 36840, 28639, 35740, 10085, 36182, 32273, 20984, 28711, 8388, 19062, 30805, 24798, 27538, 10407, 4428, 37519, 17493, 9907, 4675, 11042, 21499, 28384, 37965, 830, 6153, 26509, 20513, 19957, 21026, 25026, 36738, 12490, 2899, 66, 32386, 31863, 4822, 2688, 13508, 30003, 5538, 8373, 14372, 2377, 24347, 19235, 21901, 11150, 25495, 15098, 31445, 14876, 37611, 26570, 4203, 7145, 4133, 2935, 16082, 29001, 8242, 22146, 5637, 36998, 21470, 31665, 34704, 38125, 4973, 31343, 23569, 13514, 10926, 34025, 24958, 33734, 2852, 29614, 35764, 13286, 9636, 22693, 16159, 3834, 24074, 26260, 76, 12548, 22561, 10379, 17467, 15677, 29709, 31184, 22068, 22742, 14156, 437, 33206, 36517, 185, 36844, 6394, 18392, 6768, 20296, 12524, 15113, 15409, 26167, 2831, 37017, 5198, 20921, 9792, 34800, 11391, 37164, 3323, 21801, 809, 34557, 10862, 23806, 1769, 20573, 36525, 9857, 13490, 22680, 721, 14831, 34330, 25606, 22968, 28247, 27072, 36222, 955, 4985, 15205, 12113, 9644, 2330, 8250, 297, 37885, 5746, 28127, 27670, 9745, 26075, 3407, 11442, 37615, 12201, 34699, 5541, 585, 5427, 6211, 12845, 26212, 8471, 10749, 16425, 37601, 18911, 13933, 31740, 24519, 19649, 17695, 26080, 7153, 34870, 3935, 35106, 16556, 22843, 23968, 2190, 5946, 27225, 14820, 732, 2334, 26736, 25721, 28426, 16790, 29288, 28043, 17336, 9050, 18839, 1839, 21792, 35139, 27325, 37806, 33571, 27594, 35806, 22396, 12474, 26432, 5887, 23846, 15261, 21611, 17000, 32878, 852, 10768, 25316, 9454, 14702, 12777, 5230, 13757, 7223]

offsets500=offsets1000[:500]

largelog="../logfiles/Waller-y2014-m02-d28-h17-m21-s17.logfile"

def general_image_extractor(multiplier):
    def extract_vals_multiplier(x,imstart,m=multiplier):
        vals=P.fromstring(x[-1],P.uint8)
        vals-=48 #'0'
        vals*=multiplier 
        return vals
    return extract_vals_multiplier

def extract_vals_500(x,imstart):
    #print "last",x[-1]
    #print "seen", x[imstart]
    vals=P.fromstring(x[-1],P.uint8)
    vals-=48 #'0'
    vals*=8  # << 3
    return vals

def extract_vals_100(x,imstart):
    vals=P.array([int(v) for v in x[imstart:]],P.uint8)
    vals*=16
    return vals

#import see_log as see
large_parser=("image",extract_vals_500,offsets500,.505)
small_parser=("im000",extract_vals_100,see.Roffsets,.6)

def large_movie_from_logfile(logfile,imageprefix,moviename,parser=large_parser,start_time=0.3,end_time=30):
    import see_log as see
    import utils

    imagelabel, extractor, offsets, pavlov_baseline=parser
    ysize,xsize=120,160
    #setup offsets
    lookup=makelookups(offsets);
    labels=see.get_labels(logfile)
    imstart=labels.index(imagelabel)
    #name_inds=[(n,labels.index(n)) for n in ("iteration","r","act","prediction","beh_mode")]
    name_inds=[(n,labels.index(n)) for n in ("netTime","r","act","prediction","beh_mode","DriveCmd")]
    #Load lines at 33ms
    rs=[]
    acts=[]
    preds=[]
    behs=[]
    count=0
    step=.1
    first_time=None
    now=start_time
    end=end_time
    f=open(logfile)
    utils.run_command("rm "+imageprefix+"*.jpg")
    for line in f.readlines():
        if line[0]=="#":
            continue
        x=line.split()#[float(x for x in line.split()]
        items=[(n,float(x[a])) for n,a in name_inds]
        rs=[items[1][1]]+rs[:99]
        acts=[items[-1][1]*.25]+acts[:99]
        #print acts
        preds=[items[3][1]]+preds[:99]
        behs=[ite(items[4][1]==3, 1 , 0)  ]+behs[:99]
        #behs=[items[4][1] *.25]+behs[:99]
        vals=extractor(x,imstart)
        #vals=P.fromstring(x[imstart],P.uint8)
        #vals-=48 #'0'
        #vals*=8  # << 3
        
        #time=items[0][1]*.03
        if first_time==None:
            if items[4][1] <0:  #initial pause time
                continue
            first_time=items[0][1]
            print(first_time)
            continue
        time=items[0][1]-first_time
        if time > end:
            break
        print(now,time,count)
        while now==0 or time > now:
            now+=step
            yuvimg=fill_offset_vals(vals,lookup,(ysize,xsize))
            rgb=rgbFromyuv422Fast(yuvimg)
            data=[]
            data.append(acts)
            bb=acts[0]*4
            name=ite(bb==0, "(Drive)", ite(bb==4, "(Stop)",ite(bb==3,"(Reverse)","(Turn)")))
            data.append("motors=%d %s"%(bb,name))

            data.append(rs)
            name=ite(rs[0]==0,"(No Bump)" , "(Bump)")
            data.append("bumper=%d %s"%(rs[0],name))
            if pavlov_baseline:
                data.append([[pavlov_baseline]]+preds)            
            else:
                data.append(preds)
            data.append("prediction=%4.3f"%preds[0])
            data.append(behs)
            #bb=behs[0]*4
            #name=ite(bb==0,"Drive",ite(bb==1,"Pause",ite(bb==2,"Turn",ite(bb==3,"Test","Other"))))
            data.append("testing prediction=%d"%behs[0])
            #data=[("%s=%4.3f"%(n,v),(255,255,255)) for n,v in items]
            data.append([])
            data.append("time=%4.2f seconds"%time)

            image_annotate_save(rgb,data,imageprefix+"%06d.jpg"%count,showPred=preds[0],isBump=rs[0],showAct=acts[0]);
            count+=1
    f.close()
    #ffmpeg the results
    utils.run_command("ffmpeg  -y -r 10 -i "+imageprefix+"%06d.jpg  -sameq "+ moviename)

m20="/Users/jmodayil/data/walls/demoWaller2-y2014-m06-d09-h17-m20-s30.logfile"
m28="/Users/jmodayil/data/walls/demoWaller2-y2014-m06-d09-h17-m28-s23.logfile"
#yuv.large_movie_from_logfile("Waller-y2014-m01-d27-h17-m02-s41.logfile","/tmp/imx","/tmp/visualLearningSmall.mp4",parser=yuv.small_parser,start_time=20,end_time=300)
#yuv.large_movie_from_logfile("/Users/jmodayil/Dropbox/raspi/logfiles/Waller-y2014-m03-d04-h13-m36-s55.logfile","/tmp/imy","/tmp/visualLearningLarge.mp4",end_time=300)

