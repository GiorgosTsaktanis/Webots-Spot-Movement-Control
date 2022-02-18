"""my_controller controller."""

from controller import *
from math import *
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from maptest import *

motor_names = [
  "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
  "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
  "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
  "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"]



led_names = ["left top led",          "left middle up led", "left middle down led",
                                                "left bottom led",       "right top led",      "right middle up led",
                                                "right middle down led", "right bottom led"]


#Επιστρέφει την θέση των motors είτε σε μορφή dict είτε σε μορφή λίστας (Χωρίς όρισμα επιστρέφει dict)

#dict  -- target={"rear motor":0.5,....}
def get_current_motor_position(return_as="dict"):

    if return_as=="dict":
        current_position = dict()
        for motor_name in motor_names:
            current_position[motor_name] = motors[motor_name].getTargetPosition()
    if return_as=="list":
        current_position = []
        for motor_name in motor_names:
            current_position.append(motors[motor_name].getTargetPosition())
    

    return current_position    

#Παίρνει ως είσοδο τιμές για τα motors σε μορφ΄΄η πίνακα και επιστρέφει το dictionary.
def make_target_dict(vals):
    target_dict = dict()
    for i,motor_name in enumerate(motor_names):
        target_dict[motor_name] = vals[i]
    return target_dict    
        
#Target η θέση που θα έχουν τα motors --> dictionary 
# π.χ target["rear right shoulder abduction motor"] = 3.2
def movement_decomposition(target,duration):

    time_step = robot.getBasicTimeStep()
    #βήματα για να φτάσω στην θέση που θέλω
    n_steps = int(duration*1000/time_step)
    
    current_position = dict()
    step_difference = dict()
    for motor_name in motor_names:
        current_position[motor_name] = motors[motor_name].getTargetPosition()
        step_difference[motor_name] = (target[motor_name] - current_position[motor_name])/n_steps
    
    for i in range(n_steps):
        for motor_name in motor_names:
            current_position[motor_name]+=step_difference[motor_name]
            motors[motor_name].setPosition(current_position[motor_name])
        robot.step(timestep)   
    
    
                                                   
def stand_up(duration):
   
    vals = [0.0, 0.0, 0.0,   
            0.0,  0.0, 0.0,   
            0.0, 0.0, 0.0,   
            0.0,  0.0, 0.0] 
    target = make_target_dict(vals)
    movement_decomposition(target,duration)

def lay_down(duration):
   
    vals = [0.0, 0.0, 0.0,   
            0.0,  0.0, 0.0,   
            -0.4, -0.99, 1.59,   
            0.4,  -0.99, 1.59] 
    target = make_target_dict(vals)
    movement_decomposition(target,duration)

def controller(dist,angle_error,k1=2,k2=2,k3=10):
    elbow_angle = 0.75
    
    rotation_speed = 0.1

    angle_error = constrain(angle_error,-180,180)
 

    #abduction_motors control
    #k1 παραμετρος για την ευαισθησία των abduction_motors στο στρίψιμο
    mapping1 = interp1d([-180,180],[-0.25,0.25])
    err1 = constrain(k1*angle_error,-180,180)

    ##αν η στροφη είναι πολύ μεγάλη, σταμάτα και στρίψε πιο αργά
    if dist==0 or abs(angle_error)>80:
        mapping1 = interp1d([-180,180],[-0.15,0.15])
        err1 = constrain(3*k1*angle_error,-180,180)
    
    abduction_angle = mapping1(err1)



    #k2 ευαισθησία στο angle_error
    #k3 ευαισθησία στο distance
       
    mapping2 = interp1d([0,180],[0.32,0.06])
    err2 = constrain(k2*abs(angle_error) +  k3*(1/(dist+0.001)),0,180)
    rotation_angle =mapping2(err2)
    if dist==0 or abs(angle_error)>80:
        rotation_angle = 0 
        rotation_speed = 0.1
        

    
    if dist==0 and abs(angle_error)<=1:
        elbow_angle = 0
        

    return elbow_angle,abduction_angle,rotation_angle,rotation_speed

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def walk(target,orientation = inf):
    
    #get Position
    pos = get_pos()
    #get movement error from target
    angle_error,dist = movement_error(pos,target,orientation)

    elbow_angle,abduction_angle,rotation_angle,rotation_speed = controller(dist=dist,angle_error=angle_error)

    print("====================================================")
    print("abduction_angle: ",int(100*abduction_angle)/100)
    print("rotation_angle: ",int(100*rotation_angle)/100)
    print("dist: ",int(100*dist)/100)
    print("angle error: ",angle_error)
    print("====================================================")

    

    motor_pos = get_current_motor_position()

    speed = timestep/1000
    
    
    
    
    
    #ώθηση από τα πόδια που δεν θα σηκωθούν
    motor_pos["front right shoulder rotation motor"]=-rotation_angle
    motor_pos["rear left shoulder rotation motor"]=-rotation_angle


    ##TUNE##
    motor_pos["front right shoulder abduction motor"]=-abduction_angle
    motor_pos["rear left shoulder abduction motor"]=+abduction_angle
    ##TUNE##


    movement_decomposition(motor_pos,rotation_speed)
    
    #σηκώνω τα άλλα δύο πόδια 
    motor_pos["front left elbow motor"] = elbow_angle
    motor_pos["rear right elbow motor"] = elbow_angle
    motor_pos["rear right shoulder rotation motor"]=0
    motor_pos["front left shoulder rotation motor"]=0



    ##TEST##
    motor_pos["front left shoulder abduction motor"]=0
    motor_pos["rear right shoulder abduction motor"]=0
    ##TEST##


    movement_decomposition(motor_pos,speed)



    #κατεβάζω τα πόδια  ##kanonika 0
    motor_pos["front left elbow motor"] = 0
    motor_pos["rear right elbow motor"] = 0
    movement_decomposition(motor_pos,speed)



    

    #ώθηση από τα πόδια που δεν θα σηκωθούν (ανάποδο πόδι απο την προηγούμενη φορά)
    
    motor_pos["rear right shoulder rotation motor"]=-rotation_angle
    motor_pos["front left shoulder rotation motor"]=-rotation_angle

    

    ##TUNE##
    motor_pos["front left shoulder abduction motor"]=-abduction_angle
    motor_pos["rear right shoulder abduction motor"]=+abduction_angle

    ##TUNE##
    "// κάνω το ίδιο με πάνω "
    
    movement_decomposition(motor_pos,rotation_speed)
    #σηκώνω τα άλλα δύο πόδια και τα βάζω μπροστά κατα rotation_angle
    motor_pos["front right elbow motor"] = elbow_angle
    motor_pos["rear left elbow motor"] = elbow_angle
    motor_pos["front right shoulder rotation motor"]=0
    motor_pos["rear left shoulder rotation motor"]=0

    ##TEST##
    motor_pos["front right shoulder abduction motor"]=0
    motor_pos["rear left shoulder abduction motor"]=0
    ##TEST##
    movement_decomposition(motor_pos,speed)


    #κατεβάζω τα πόδια kanonika 0 
    motor_pos["front right elbow motor"] = 0
    motor_pos["rear left elbow motor"] = 0
    
    movement_decomposition(motor_pos,speed)
    


    return dist,angle_error 
def get_pos(digits = 3,disp = False):
    
    multiplier = 10**digits
    vals = gps.getValues()
    vals = [int(multiplier*x)/multiplier for x in vals]
    out = {"x":vals[0],"y":vals[1],"z":vals[2]}
    if(disp ==True):
        print(out)

    return out
def movement_error(pos,target,target_angle=inf):


    #Που κοιτάει το ρομπότ
    current_angle = (-imu.getRollPitchYaw()[2]-pi/2)*57.29578
    
    if target_angle == inf:
        X = target["x"]-pos["x"]
        Z = target["z"]-pos["z"]
        #που πρέπει να κοιτάει
        target_angle = atan2(Z,X)*57.29578
   
    angl = (target_angle - current_angle)

    

    if angl>180:
        angl = -(360-angl)
    if angl<-180:
        angl = angl+360


   
    dist = sqrt(   (pos["x"]-target["x"])**2 +(pos["z"]-target["z"])**2)

    return (int(angl*100)/100),dist
           


if __name__=="__main__":
    # create the Robot instance.
    robot = Robot()
    
    
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    #init motors
    motors = dict()
    for motor_name in motor_names:
        motors[motor_name] = robot.getDevice(motor_name)

  
    #Enable GPS
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    #Enable IMU
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    
    
    
    once =True
    i = 0
    time = 0

    #Create targets 
    targets,img_xs,img_ys = make_path()


    img = plt.imread("edges.jpg")
    fig,ax = plt.subplots()
    line = ax.imshow(img)
    fig.show()
    fig.canvas.draw()
    ax.scatter(img_xs,img_ys,s=120,color = "red")
    ax.plot(img_xs,img_ys,color="white")
    
    real_xs = []
    real_ys = []
  
    while robot.step(timestep)!=-1:
        if once==True:
            stand_up(0.5)
            once = False
        
        pos = get_pos()
        real_xs.append(pos["x"])
        real_ys.append(pos["z"])
        img_xs,img_ys = real_to_img_coords(real_xs,real_ys)
        ax.plot(img_xs,img_ys,color="cyan")
        fig.canvas.draw()
    
        if i<len(targets):
            dist,angle_error = walk(targets[i],inf)
            if dist<0.3: i=i+1
        
        else:
            """ orientation = 50
            dist,angle_error = walk(pos,orientation)
            if dist==0 and abs(angle_error) < 1 :
                plt.show()
                print(robot.getTime())
                exit() """   
            lay_down(0.5)             
    
        
    
         
        
       
        
            

    # Enter here exit cleanup code.
