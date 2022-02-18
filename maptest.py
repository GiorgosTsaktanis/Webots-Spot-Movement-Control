import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np

def make_path():
    img = plt.imread("map.png")
    img_xs = []
    img_ys = []
    def onclick(event):
        if len(img_xs)>0 and abs(img_xs[-1]-event.xdata)<=10 and abs(img_ys[-1]-event.ydata)<=10:
            plt.close()
            return
        img_xs.append(float(event.xdata))
        img_ys.append(float(event.ydata))
        ax.scatter(img_xs,img_ys,s=120,color = "red")
        ax.plot(img_xs,img_ys,color="blue")
        line.figure.canvas.draw()
        
    fig,ax = plt.subplots()
    line = ax.imshow(img)
    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    targets = generate_targets(img_xs,img_ys)
    
    #return generate_targets(img_xs,img_ys),img_xs,img_ys
    return fix_path(targets,img_xs,img_ys)
def img_to_real_coords(img_xs,img_ys):
    x_mapping = interp1d([0,622],[-5,5])
    y_mapping = interp1d([0,622],[-5,5])
    x_real = [float(x_mapping(x)+0.03) for x in img_xs]
    y_real = [float(y_mapping(y)+0.09) for y in img_ys]
    return x_real,y_real
def real_to_img_coords(real_xs,real_ys):
    x_mapping = interp1d([-5,5],[0,622])
    y_mapping = interp1d([-5,5],[0,622])
    img_xs = [x_mapping(x-0.03) for x in real_xs]
    img_ys = [y_mapping(y-0.09) for y in real_ys]
    return img_xs,img_ys
def generate_targets(img_xs,img_ys):
    real_xs,real_ys = img_to_real_coords(img_xs,img_ys)
    targets = []
    for i in range(len(real_xs)):
        targets.append({"x":real_xs[i],"z":real_ys[i]})

    
 
    return targets

def fix_path(targets,img_xs,img_ys):

    
    img = plt.imread("edges.jpg")
    """ fig,ax = plt.subplots()
    line = ax.imshow(img)
    ax.scatter(img_xs,img_ys,s=120,color = "red")
    ax.plot(img_xs,img_ys,color="blue") """
    ##line eq  y = mx + b 
    
    img_ys[1] = round(img_ys[1])
    img_xs[1] = round(img_xs[1])
    m = (img_ys[1] - img_ys[0])/(img_xs[1] - img_xs[0])
    b = img_ys[1] - m*img_xs[1]

   
    minx =  round(min(img_xs[1],img_xs[0]))
    maxx =  round(max(img_xs[1],img_xs[0]))

    found = False
    problemx = 0
    problemy = 0
    for x in range(minx,maxx+1):
        x = round(x)
        y = round(m*x+b)
        
        
        if img[y,x]>0:
            
            found = True
            problemx = x
            problemy = y
            break
       
    if found==False:
        return targets,img_xs,img_ys
    m_ = -1/m
    b_ = problemy - m_*problemx

    xnew = problemx - 50

    ynew = round(m_*xnew + b_)
    img_xs.insert(1,xnew)
    img_ys.insert(1,ynew)

    """ ax.scatter(img_xs,img_ys,color="white")
    ax.plot(img_xs,img_ys,color="magenta") """
    targets = generate_targets(img_xs,img_ys)

    """ ax.scatter(xnew,ynew,color="green",s=20)
    line.figure.canvas.draw() """


            
    return targets,img_xs,img_ys
#print(targets)