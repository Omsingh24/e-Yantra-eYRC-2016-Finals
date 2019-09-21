
'''*Team Id : eYRC-CC#2304
* Author List : Om Singh,Rishabh Raj,Gourav SIngh ,Milanpreet singh
* Filename: eYRC-CC#2304
* Theme: Cross A Crater (eYRC #CC)
* Functions: cavity,obstacle,number,mainarea,botsrt,botend,calculate,align,movef,xbee,forward
             back,stop,right,left,pick,drop,buzzeron,buzzeroff,doworkp,doworkm,doworkd,summ
* Global Variables:   cap,endit,cavloc[],img_rgb,img_hsv,mainareacrop,numloc,obsloc[],botstart,botenda,
                    cavloc_bridge1,cavloc_bridge2,sumi,sol,bridge,numgrid,obs_bridge

    '''



import numpy as np
import cv2
import serial
import math
import time
import itertools
'''
* Function Name: cavity
* Input: hsv image
* Output: centre point of all cavity (blue) ,  cavity list
* Logic: image is converted to hsv adnd blurred to get better color profile,image is masked
    based on lower and upper range,morphological transform opening is applied to erode and dilate the mask,
    loop over all the found contours and selecting the contour based on area and appending there centre point to cavloc list


* Example Call: cavity(img_rgb)
'''

    
def cavity(img_hsv):
    #image converted to burred hsv
    img_hsv = cv2.blur(img_hsv,(5,5))
    # list to save centere coordinates
    cavity_list=[]
    #lower and upper value of blue hsv 
    lower_range = np.array([68,70,128])
    upper_range= np.array([118,255,206])
    #masking the hsv image
    
    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    kernel = np.ones((6,6),np.uint8)
    # erodind and dilating the mask
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    ret,thresh = cv2.threshold(opening,10,255,0)
    # finding contours
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)


    i=0
    
    cavloc=[]
    #loop for appending found contours in contour list
    for cnt in contours:
        M = cv2.moments(cnt)
        #mid point of contour cx,xy
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        # mid point of minenclosing circle and radius
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (cx,cy)
       
        center = (int(x),int(y))
        
        radius = int(radius)
        if radius>10 and radius<20: #limiting the radius of contour to get rid of small and big unwanted contours
            i=i+1
            cv2.line(img_rgb,(cx,cy),(cx-50,cy),(0,255,0),3)
           
            cv2.circle(img_rgb,center,radius,(0,255,0),2)
            cavloc.append(center) # appending the center pont to the list cavloc
            cavity_list.append(cnt)
                
            '''if len(contour_list)>0:
                (x,y),radius = cv2.minEnclosingCircle(contour)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(img_rgb,center,radius,(0,255,0),2)'''
            
            x,y,w,h= cv2.boundingRect(cnt)
            
            
    #returning list containg centre point of cavity

    return cavloc

'''
* Function Name: obstacle
* Input: hsv image
* Output: centre point of all obstacle(green) , obstacle list
* Logic: image is converted to hsv and blurred to get better color profile,image is masked
    based on lower and upper range,morphological transform opening is applied to erode and dilate the mask,
    loop over all the found contours and selecting the contour based on area and appending there centre point to obstacle list(obsloc)


* Example Call: cavity(img_rgb)
'''

def obstacle(img_hsv):
    #hsv blurred
    img_hsv = cv2.blur(img_hsv,(5,5))
    
    #lower and upper hsv value for green color
    
    lower_range = np.array([62,180,0])
    upper_range= np.array([119,255,96])

    kernel = np.ones((5,5),np.uint8)
# lower and upper value masked
    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    #erode and dilate morphologial transform opening
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    ret,thresh = cv2.threshold(opening,20,255,0)

    #finding all possiblecontour
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)



    obsloc=[]
    #iteration through all contours
    for contour in contours:
        
        approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
    
        if (area > 10) : # limiting all contours above area 10 
          


        
            M = cv2.moments(contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            obsloc.append([cx,cy])
            
            x,y,w,h = cv2.boundingRect(contour)
            
            cv2.rectangle(img_rgb,(x,y),(x+w,y+h),(0,0,255),2)
            
        
            # return a list containg the center point of all obstacle 
    return obsloc
'''
* Function Name: number
* Input: rgb image
* Output: center point of all the matched temlate in list ,index correspond to the number detected
* Logic: image is conveted to grayscale ,templates are saved in list and iterated and matched with image
* Example Call: number(img_rgb)
'''


def number(img_rgb):
    #converting image to grayscale
    
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY) 


    
    
    k = cv2.waitKey(42) & 0xFF
    #dimension ig image
    
    b,l=img_gray.shape
    #cropping image to 1/3 of width

    half=img_gray[0:b,0:l/3]
    
    
    k = cv2.waitKey(42) & 0xFF 
    centerx=0
    centery=0
   # template images
    template_list=['4.jpg','5.jpg','7.jpg','3.jpg']
# list to save center point of the matched template
    locarray=[0,0,0,0,0,0,0,0,0,0,0]
    #loop for all template images
    for j in template_list:
        temp=cv2.imread('digit/'+j,0)
        #main template function for template matching
        res = cv2.matchTemplate(half,temp,cv2.TM_CCOEFF_NORMED)

        # max valueof match and loc of teplate matched
        _,max_val,_,loc= cv2.minMaxLoc(res)
        
        
        
        '''if (j=='8.jpg') and max_val>.7:
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(0,0,255),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[8]=([centerx,centery])'''
        if (j=='7.jpg') and max_val>0.6:
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(255,255,0),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[7]=([centerx,centery])
        '''if (j=='0.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(0,255,0),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[0]=([centerx,centery])'''
        '''if (j=='1.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(255,255,255),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[1]=([centerx,centery])'''
        '''if (j=='00.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(255,0,255),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[10]=([centerx,centery])'''
        if (j=='3.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(255,0,0),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[3]=([centerx,centery])
        if (j=='4.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(0,255,0),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[4]=([centerx,centery])
        if (j=='5.jpg') :     
            
            cv2.rectangle(img_rgb,(loc[0],loc[1]),(loc[0]+34,loc[1]+36),(0,0,255),2)
            centerx=(loc[0]+loc[0]+34)/2
            centery=(loc[1]+loc[1]+36)/2
            locarray[5]=([centerx,centery])
        

        
        

# returning list of centere point of matched template ,where ndex correspond to the matched number
    
    return locarray



'''
* Function Name: mainarea
* Input: hsv image
* Output: x,y starting coordinate and widht and heeight of the main area contour
* Logic: limiting the area of contour of black color gives the cropped arena
* Example Call: mainarea(img_rgb)
'''



def mainarea(img_hsv):
    # hsv image blurred
    img_hsv = cv2.blur(img_hsv,(5,5))
    
    # lower and upper range of black color hsv
    
    lower_range = np.array([0,0,0])
    upper_range= np.array([169,196,194])

    kernel = np.ones((5,5),np.uint8)
#masked black
    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    #eroding and dilating
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    ret,thresh = cv2.threshold(opening,127,255,0)

#finding contourr
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)


    
    mainareacrop=[]
#iterating though all th e found contours
    for contour in contours:
        
        approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
        area = cv2.contourArea(contour)
    
        if (area >100000)and area<250000 :# limiting the area to get second largest contour
            

    
  

            
            x,y,w,h = cv2.boundingRect(contour)
            mainareacrop.append([x,y,w,h])
            cv2.rectangle(img_rgb,(x,y),(x+w,y+h),(0,0,255),2)
    #returninfg x,y,w,h i.e x,y starting and width and height of mainarea contour        
    
    return mainareacrop



'''
* Function Name: botsrt
* Input: hsv image
* Output: center point of face marker (green color)
* Logic: green color is placed in front of bot to get the face point of the bot
* Example Call: botsrt(img_hsv)
'''
def botsrt(img_hsv):
    #image blurred
    img_hsv = cv2.blur(img_hsv,(6,6))
    # lower and upper range of green color hsv 
    lower_range = np.array([40,77,119])
    upper_range= np.array([61,255,255])

    

    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    

    kernel = np.ones((6,6),np.uint8)

   # erodinng anfd ilating the mask
    
    opening= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    
    
     
    ret,thresh = cv2.threshold(opening,20,255,0)
    #finding countours in mask
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    
    
   
   
    botsrt=[(0,0)]
    #iteratinf throus all found contours
    
    for cnt in contours:
       
            
        M = cv2.moments(cnt)
        if M['m00']>0:
            cx =int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center =(cx,cy)
            
            #center = (int(x),int(y))
            radius = int(radius)
            
            if radius<150 :   # limiting contours greater than 150 radius
                botsrt[0]=center
               
                cv2.line(img_rgb,(cx,cy),(cx,cy),(0,0,255),3)
                
                cv2.circle(img_rgb,center,radius,(0,255,0),2)
                
     #return center oint of green marker place on face of bot

    return botsrt




'''
* Function Name: botend
* Input: hsv image
* Output: center point of back marker (orange color)
* Logic: orange color is placed in front of bot to get the back point of the bot
* Example Call: botend(img_hsv)
'''
def botend(img_hsv):
    # lower and upper range of orange hsv 
 
    
    lower_range = np.array([0,144,196])
    upper_range= np.array([179,211,255])

    
#maked
    mask = cv2.inRange(img_hsv, lower_range, upper_range)
    

    kernel = np.ones((5,5),np.uint8)
    
    
   # eroding and dilating the contours
    erosion = cv2.erode(mask,kernel,iterations = 1)
    dilatee = cv2.dilate(erosion,kernel,iterations = 2)

    
    
    
    ret,thresh = cv2.threshold(dilatee,20,255,0)


    # finding alll contours
    contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

    
    
   

    botend=[(0,0)]
    #iterating all found contours
    for cnt in contours:
        M = cv2.moments(cnt)
        
        cx =int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
        
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center=(cx,cy)
        
        #center = (int(x),int(y))
        radius = int(radius)
        
        if radius<150:  #limiting all contours greater than 150
            botend[0]=center

    
            cv2.line(img_rgb,(cx,cy),(cx,cy),(0,0,255),3)

            
# draing circle on the mid point
            cv2.circle(img_rgb,center,radius,(0,255,0),2)
            
    
    #return the center point of orange marker   
    return botend
    
'''
* Function Name: calculate
* Input: botstar,botend,dummy
* Output: angle and distance beteen the dummy point and the bot axis
* Logic: angle and distance is calculated usinfg the center point of orange and green markers on bot and distanvce between the dummy point which can be a cavity cnter point or obstacle center point

* Example Call: calculate(botstart,botend,dummy)
'''




def calculate(botsrt,botend,dummy):
    
    all(i>0 for i in botsrt)
    True
        
        
    #marking the dummy point
    cv2.circle(img_rgb,dummy,2,(0,0,255),2)


    # makin a line from bot start to bot end
    cv2.line(img_rgb,botsrt[0],botend[0],(0,0,255),3)
    # calculating center point of cot using botstrt and bot end markers center
    center_bot_y=((botsrt[0][1]+botend[0][1]))/2
    center_bot_x=((botsrt[0][0]+botend[0][0]))/2
    
    center_bot=(center_bot_x,center_bot_y)
    cv2.circle(img_rgb,center_bot,4,(0,255,255),2)

    cv2.line(img_rgb,center_bot,dummy,(0,0,255),3)

    

    
    x3=dummy[0]
    y3=dummy[1]
    #pick 1
    x2=center_bot[0]
    y2=center_bot[1]
    
   
    
    # slope of center of bot  and dummy point
    m1=math.degrees(math.atan2( (y2-y3),(x2 - x3)))
    m1=int(m1)
    #slope of bor strat green marer and center of bot

    m2=(math.atan2( y2-botsrt[0][1],x2-botsrt[0][0]) * 180.0 / math.pi)

    
    # angle between center of bot and dummy point
    
    m2=int(m2)
    angle=(m1-m2)
    if (angle)<0:
        #print 'angle',(angle)+360
        angle=angle+360
    else:
        #print 'angle' ,(angle)
        angle=angle
    #if (m2<0):
        #m2=-(m2)
    #print y2,y3,x2,x3
    if y2>0 and y3>0 and x2>0 and x3>0 :
        
        dist=(((y2-y3)**2)+((x3-x2)**2))**.5 # diatnce bewteen center and and dummy point
     
        dist=int(dist)
    else :
        dist=111111  
        

#returns the distance angle and center of bot coordinates

        
    return dist,angle,center_bot
'''
* Function Name: align
* Input: angle
* Output: 1 if aligned or 0 if not aligned
* Logic: it checkes whether the bot is aligned or not if aligned returns a value 1 and if not aligned align it using right or left function

* Example Call: align(angle)
'''



            
def align(angle):
# angle between the ais of bot and dummy point

    if angle==0 or angle>353 or angle<7:
        print 'aligned'
        return 1
    elif angle>180 :
        left(ser)
        print 'left'
        return 0
    elif angle<180:
        right(ser)
        print 'right'
        return 0
    
    
    
        

'''
* Function Name: movef
* Input: dist
* Output: callers the forward function
* Logic: make the bot move forward
* Example Call: botend(img_hsv)
''' 
def movef(dist):
    forward(ser)
         
'''
* Function Name: xbee
* Input: 
* Output: opens the com port where xbee is connecteed
* Logic:Com port is opend to send data to xbee on bot
* Example Call: xbee()
'''
def xbee():
    ser = serial.Serial(port='COM14',baudrate=9600)
    
    return ser

'''
* Function Name: forward
* Input: ser
* Output: sends '8' to xbee on bot
* Logic: send data to xbee on bot
* Example Call: botend(img_hsv)
'''
def forward(ser):
    ser.write('8')
def back(ser):   # makes the bot move back
    ser.write('2')
def stop(ser):       # makes the bot stop
    ser.write('5')
def right(ser): # makes the bot move right
    ser.write('6')
def left(ser): # makes the bot move let
    ser.write('4')
def pick(ser): # makes the bot pick
    ser.write('7')
def drop(ser): # makes the bot drop
    ser.write('9')
def buzzeron(ser): # makes the bot buzzer on
    ser.write('1')
def buzzeroff(ser): # makes the bot buxxer off
    ser.write('3')



'''
* Function Name: doworkp
* Input: angle,dist
* Output: call  the align function and movef function and pick function while moving to number bouldr 
* Logic: using angle and distance the bot moves to the boulder and stops when the distance is less than 98  this function is only called when dummy is set to boulder
* Example Call: doworkp(angle,dist)
'''

 
def doworkp(angle,dist):
    
    if (angle==0) or angle>353 or angle<7:
        
        ac=1
    else:
        align(angle)
        ac=0
        
    if  (ac==1) and (dist<98) or (dist>500):
        
        print 'stop'
        stop(ser)
    
        
    if (ac==1 and dist>98):
        print'move'
        pickit=0
        movef(dist)
    else:
        pickit=1
   
        
    if (dist<98) and (ac==1) and pickit==1:
        print 'pick'
        pick(ser)
        for i in range(0,30):
            back(ser)

        
        
        return 1
        
'''
* Function Name: doworkm
* Input: angle,dist
* Output: call  the align function and movef function and pick function while moving to midpoints  
* Logic: using angle and distance the bot moves to the dummy pint and stops when the distance is less than 25 this function is only called when dummy is not on boulder or cavity
* Example Call: doworkm(angle,dist)
'''     
def doworkm(angle,dist):
    
    if (angle==0) or angle>353 or angle<7:
        
        ac=1
    else:
        align(angle)
        ac=0
        
    if  (ac==1) and (dist<25) or (dist>500):
        
        print 'stop'
        stop(ser)
        return 1
    
        
    if (ac==1 and dist>25):
        print'move'
        movef(dist)
    
    #if botloc[1]==dummy:
     #   stop();
      #  print 'fstop'

    print angle,dist,ac,

        
'''
* Function Name: doworkd
* Input: angle,dist
* Output: call  the align function and movef function and drop function while moving to cavity
* Logic: using angle and distance the bot moves to the dummy point and stops when the distance is less than 105 this function is only called when dummy is on cavity
* Example Call: doworkm(angle,dist)
'''     
def doworkd(angle,dist):
    
    if (angle==0) or angle>353 or angle<7:
        
        ac=1
    else:
        align(angle)
        ac=0
        
    if  (ac==1) and (dist<105) or (dist>500):
        
        print 'stop'
        stop(ser)
    
        
    if (ac==1 and dist>105):
        print'move'
        dropit=0
        movef(dist)
    else :
        dropit=1
    if (dist<105) and (ac==1) and dropit==1:
        print 'drop'
        drop(ser)
        return 1

        
'''
* Function Name: sumi
* Input: numgrid,sumi,cavloc_bridge1,cavloc_bridge2
* Output: returns the list with possible sum combination from numgrid annd vakue of cavity in bridge 2
* Logic: using angle and distance the bot moves to the dummy pint and stops when the distance is less than 25 this function is only called when dummy is not on boulder or cavity
* Example Call: doworkm(angle,dist)
'''     

def summ(numgrid,sumi,cavloc_bridge1,cavloc_bridge2):

    # choses the bridge whose cavity are less

    if len(cavloc_bridge1)>len(cavloc_bridge2):
        a=len(cavloc_bridge2)
    else:
        a=len(cavloc_bridge1)

    
    

    


    # gives a list with possible combination with sum save in variable sumi
    result = [seq for i in range(len(numgrid), 0, -1) for seq in itertools.combinations(numgrid, i) if sum(seq) == sumi]
    print result
    
    
    if len(result)==0:
        print' No solution'
        return 0
    else:
        for i in result:
            len(i)
            
            if len(i)==a:
                
                return i
                break
            '''if len(i)==2:
                
                return (i[0],i[1])
                break
            if len(i)==4:
                
                return (i[0],i[1],i[2],i[3])
                break'''
    





    ####################################main function################
cap=cv2.VideoCapture(0)# camera on

ser=xbee()#xbee on
endit=0  #endit flag to check if the all cavity are filled or not
cavloc=[]



for i in range(0,10): #capturing 10 frames and cropping the main area 
    
    _,img_rgb=cap.read()
    img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    mainareacrop=mainarea(img_rgb,img_hsv)
    
    if (len(mainareacrop)==1):
          
          x=mainareacrop[0][0]
          y=mainareacrop[0][1]
          w=mainareacrop[0][2]
          h=mainareacrop[0][3]
          img_rgb=img_rgb[y:y+h,x:x+w]# cropping
          
    k = cv2.waitKey(42) & 0xFF
    if k == 27:
        ser.close()
        break



for i in range(0,10):  # iterating 10 frames and finding the cavity obstacle and boulder botstart and bot end point
     
     
     _,img_rgb=cap.read()
     
     img_rgb=img_rgb[y:y+h,x:x+w]  # cropping man area
        
     img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)  #converting rgb to hsv
     

     obsloc=obstacle(img_hsv) # calling obstacle tacle function and saves the mid point of all obstacle in obsloc variable list
     
         
     cavloc=cavity(img_rgb_blur,img_hsv) # calling cavity function and saves the mid point of all cavity in cavloc variable list

        
     numloc=number(img_rgb) #calling numberfunction and saves the mid point of all boulder in numloc variable list



     botstart=botsrt(img_hsv) #call the botsrt function and saves the green marker mid point in botstart variable

     botenda=botend(img_hsv) ##call the botsrt function and saves the orange marker mid point in botend variable
     
     
     cv2.imshow('main',img_rgb)
     k = cv2.waitKey(42) & 0xFF
     if k == 27:
         ser.close()
         break
#sorting the cavity ponts in ascending order along x axis
for elm in cavloc:                                                              
    cavloc.sort(key=lambda x: x[0])


cavloc_bridge1=[]
cavloc_bridge2=[]
# sorting the cavity points if above the mid point then cavity in ridge 2 and if below mid point bridge 1
for ele in cavloc:
   
    i=0
    if ele[1]>240:
        cavloc_bridge1.append(ele)
    if ele[1]<240:
        cavloc_bridge2.append(ele)
    i=i+1
    

print 'bridge1',cavloc_bridge1
print 'bridge2',cavloc_bridge2

obs_bridge=[]
#sorting obstacle based on x axis location
for elm in obsloc:                                                              
    obsloc.sort(key=lambda x: x[0])

for i in obsloc:
    
    if i[1]<150:
        obs_bridge.append(i)

            

print 'obstacle in bridge',obs_bridge

print numloc


numgrid=[]
j=0

#iterating values in numloc and save index values where template is found i.e 
for i in numloc:
    
    if i>0:
        
        numgrid.append(j)

    j=j+1
    
print numgrid
sumi=input('Enter sum') #user input the sum
if j>9 and (len(cavloc_bridge2) or len(cavloc_bridge1))>2:   #if number on boulder occur twice the second number is saved in 10-1 index so the sum is incresed by 10
    sumi=sumi+10
    sol=summ(numgrid,sumi,cavloc_bridge1,cavloc_bridge2)
else:
    sol=summ(numgrid,sumi,cavloc_bridge1,cavloc_bridge2)

print sol
if len(sol)==0:
    print 'No Combination'
    
if len(sol)==len(cavloc_bridge1):
    print 'Bridge 1'
    bridge=1#selecting bridge based on cavity present in bridge and combination 

if len(sol)==len(cavloc_bridge2):
    print 'bridge 2'
    bridge=2

index=0

#################bridge 1#################################################################################

if bridge==1:
    print 'selected bridge 1'
    for i in sol:
         #if i>9:
          #   i==
             
         b1m=(cavloc_bridge1[0][0]-100,cavloc_bridge1[0][1])
         cv2.circle(img_rgb,b1m,4,(0,255,255),2)
        
         
         
         
         ##the while loop runs untilt the bot reaches the boulder and picks it up####
         while(1):
             time.sleep(.1)
             
             
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)

             botstart=botsrt(img_hsv)

             botenda=botend(img_hsv)
             
             dummy=(numloc[i][0],numloc[i][1])   #dummy point set to boulder midpoint
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkp(angle,dist)
             print angle,dist
             if chk==1:
                  break
            
             cv2.imshow('out',img_rgb)
             b1e=(cavloc_bridge1[3][0]+10,cavloc_bridge1[3][1])
             endp=(b1e[0]+50,b1e[1]-100)
             cv2.circle(img_rgb,endp,2,(0,255,255),2)
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break

         ##the while loop runs untilt the bot reaches the mid point ####        
         while(1):
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
             
            
                 
             b1m=(cavloc_bridge1[0][0]-100,cavloc_bridge1[0][1]) #dummy point set to midpoint b1m
             cv2.circle(img_rgb,b1m,4,(0,255,255),2)
                
            
             
             
             
             botstart=botsrt(img_hsv)
             
             botenda=botend(img_hsv)
            
             dummy=(b1m[0],b1m[1])##dummy point set to midpoint b1m
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break
    ##the while loop runs untilt the bot reaches the cavity and drops the boulderin it ###
         while(1):
             
             
            
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
             
            
             
             
             
             botstart=botsrt(img_hsv)
         
             botenda=botend(img_hsv)
             
             dummy=(cavloc_bridge1[index][0],cavloc_bridge1[index][1]) #dummy point set to cavity mid point
             if index==(len(sol)-1): #endit flag is raised when index=length of solution -1
                  endit=1
                                  
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkd(angle,dist)
             print angle,dist
             if chk==1:
                  break


              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break


         ##if the endit flag is not raised the bot returns to mid point and then picks other boulder else it moves forward  ###
         while(1):
             if endit==1:
                 break
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
             
                         
             botstart=botsrt(img_hsv)
             
             botenda=botend(img_hsv)
            
             dummy=(b1m[0],b1m[1])  #dummy point set to midpoint b1m
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break
         index=index+1
         
         #######3bot moves to bridge 1 end mid point to reach  base point######
    if endit==1:
        while(1):
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
             
             b1e=(cavloc_bridge1[3][0]+10,cavloc_bridge1[3][1])
             
                 #endp=
             
             cv2.circle(img_rgb,b1e,4,(0,255,255),2)
             
             
             
             
             botstart=botsrt(img_hsv)
             
             botenda=botend(img_hsv)
            
             dummy=(b1e[0],b1e[1])  #dummy point set to end mid point
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                ser.close()
                break
    ###finally the bot goes to the the end base pont#########
        while(1):
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
             
             b1e=(cavloc_bridge1[3][0]+10,cavloc_bridge1[3][1])    
             endp=(b1e[0]+50,b1e[1]-100)

             
             cv2.circle(img_rgb,b1e,4,(0,255,255),2)
             
            
                 
             
             botstart=botsrt(img_hsv)
             
             botenda=botend(img_hsv)
            
             dummy=(endp[0],endp[1])     #dummy point set to final end point
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  buzzeron(ser)
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break
    
    cv2.destroyAllWindows()




###############################################bridge 2##############################################################

if bridge==2:
    print 'selected bridge 2'

    b2m=(183,100)
    
    for i in sol:
         
         print index
         print 'pick',i
         
  #################pick up boulder#######################       
         while(1):

             print 'pick'
             time.sleep(.1)
             print 'numloc',numloc[i]
             
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)

             botstart=botsrt(img_hsv)

             botenda=botend(img_hsv)
             
             dummy=(numloc[i][0],numloc[i][1])   #dummy point set to boulder
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,(numloc[i][0],numloc[i][1]),15,(0,255,0),3)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkp(angle,dist)
             print angle,dist
             if chk==1:
                  
                  
                  time.sleep(.1)
                  break
                
 
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 for i in range(0,20):
                    back(ser)
                 ser.close()
                 break
         ################



    ##################center mid############
         while(1):
             print 'center mid run'
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
             
             
             botstart=botsrt(img_hsv)
         
             botenda=botend(img_hsv)
             
             dummy=(162,185)##dummy point set to center mid
             
             if index==(len(sol)-1):
                 endit=1
                                  
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
           
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break
        


         while(1):
             print 'mid run'
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
             
             
             botstart=botsrt(img_hsv)
         
             botenda=botend(img_hsv)
             
             dummy=(b2m[0],b2m[1])  ###dummy point set to bridge 2 mid
             
             if index==(len(sol)-1):
                 endit=1
                                  
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             
             chk=doworkm(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break
        
#### obs loc####################

         if index==1:
            while(1):
                
                 

                 
                print 'obs'
                time.sleep(.1)
                _,img_rgb=cap.read()
                img_rgb=img_rgb[y:y+h,x:x+w]
                
                img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
                 
                 
                botstart=botsrt(img_hsv)

                botenda=botend(img_hsv)
                 
                dummy=(obs_bridge[0][0],obs_bridge[0][1]-80) 
                 
                if index==(len(sol)-1):
                    
                    endit=1
                                      
                m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
                cv2.circle(img_rgb,dummy,2,(0,255,255),2)
                chk=doworkm(angle,dist)
                print angle,dist
                if chk==1:
                    break

                  
                 
                 
                cv2.imshow('out',img_rgb)

                 
                k = cv2.waitKey(42) & 0xFF
                if k == 27:
                    ser.close()
                    break
########################drop##################
         while(1):

             print 'drop'
             time.sleep(.1)
             _,img_rgb=cap.read()
             img_rgb=img_rgb[y:y+h,x:x+w]
             img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
             
             
             botstart=botsrt(img_hsv)
         
             botenda=botend(img_hsv)
             
             dummy=(cavloc_bridge2[index][0],cavloc_bridge2[index][1])
             
             if index==(len(sol)-1):
                 endit=1
                                  
             m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
             cv2.circle(img_rgb,dummy,2,(0,255,255),2)
             chk=doworkd(angle,dist)
             print angle,dist
             if chk==1:
                  break

              
             
             
             cv2.imshow('out',img_rgb)

             
             k = cv2.waitKey(42) & 0xFF
             if k == 27:
                 ser.close()
                 break

             
             

    ################return mid run###############
         if endit!=1:
             
                 
                 
             while(1):
                 
                 
                 print 'mid run'
                 time.sleep(.1)
                 _,img_rgb=cap.read()
                 img_rgb=img_rgb[y:y+h,x:x+w]
                 img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
                 
                 
                 botstart=botsrt(img_hsv)
             
                 botenda=botend(img_hsv)
                 
                 dummy=(b2m[0],b2m[1])
                 
                 if index==(len(sol)-1):
                     endit=1
                                      
                 m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
                 cv2.circle(img_rgb,dummy,2,(0,255,255),2)
                 
                 chk=doworkm(angle,dist)
                 print angle,dist
                 if chk==1:
                      break

                  
                 
                 
                 cv2.imshow('out',img_rgb)

                 
                 k = cv2.waitKey(42) & 0xFF
                 if k == 27:
                     ser.close()
                     break

############################return center run#################
         if endit!=1:
             
                 
             while(1):
                 
                 
                 print 'center mid run'
                 time.sleep(.1)
                 _,img_rgb=cap.read()
                 img_rgb=img_rgb[y:y+h,x:x+w]
                 img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
                 
                 
                 botstart=botsrt(img_hsv)
             
                 botenda=botend(img_hsv)
                 
                 dummy=(162,185)
                 
                 if index==(len(sol)-1):
                     endit=1
                                      
                 m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
                 cv2.circle(img_rgb,dummy,2,(0,255,255),2)
                 
                 chk=doworkm(angle,dist)
                 print angle,dist
                 if chk==1:
                      break

                  
                 
                 
                 cv2.imshow('out',img_rgb)

                 
                 k = cv2.waitKey(42) & 0xFF
                 if k == 27:
                     ser.close()
                     break






         index=index+1
         print index
###################endit#################
    if endit==1:
        while(1):
            
            
            
            time.sleep(.1)
            _,img_rgb=cap.read()
            img_rgb=img_rgb[y:y+h,x:x+w]
            img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
            obsp1=(obs_bridge[1][0],obs_bridge[1][1]+60)
             
            cv2.circle(img_rgb,obsp1,4,(255,255,255),2)
            
             
             
             
            botstart=botsrt(img_hsv)
             
            botenda=botend(img_hsv)
            
            dummy=(obsp1[0],obsp1[1])
            m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
            cv2.circle(img_rgb,dummy,2,(0,255,255),2)
            chk=doworkm(angle,dist)
            print angle,dist
            if chk==1:
                break



            cv2.imshow('out',img_rgb)

             
            k = cv2.waitKey(42) & 0xFF
            if k == 27:
                ser.close()
                break
     ##########################bridge2 end base mid point################   
        while(1):
            
            
            time.sleep(.1)
            
            _,img_rgb=cap.read()
            img_rgb=img_rgb[y:y+h,x:x+w]
            img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
            endp1=(516,71)
             
            cv2.circle(img_rgb,endp1,4,(255,255,255),2)
            
             
             
             
            botstart=botsrt(img_hsv)
             
            botenda=botend(img_hsv)
            
            dummy=(endp1[0],endp1[1])
            m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
            cv2.circle(img_rgb,dummy,2,(0,255,255),2)
            chk=doworkm(angle,dist)
            print angle,dist
            if chk==1:
                break



            cv2.imshow('out',img_rgb)

             
            k = cv2.waitKey(42) & 0xFF
            if k == 27:
                ser.close()
                break
######################end point run####################

        while(1):
            
            
            time.sleep(.1)
            
            _,img_rgb=cap.read()
            img_rgb=img_rgb[y:y+h,x:x+w]
            img_hsv=cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
            
         
            endpf=(520,160)
             
            cv2.circle(img_rgb,endpf,4,(255,255,255),2)
            
             
            botstart=botsrt(img_hsv)
             
            botenda=botend(img_hsv)
            
            dummy=(endpf[0],endpf[1])
            m1,m2,dist,angle,ceneter_bot=calculate(botstart,botenda,dummy)
            cv2.circle(img_rgb,dummy,2,(0,255,255),2)
            chk=doworkm(angle,dist)
            print angle,dist
            if chk==1:
                
                
                break

            

            cv2.imshow('out',img_rgb)

             
            k = cv2.waitKey(42) & 0xFF
            if k == 27:
                
                break      

        


print 'Task Complete'
buzzeron(ser)



       



    

