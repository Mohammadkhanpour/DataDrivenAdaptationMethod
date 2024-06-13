from Sw_Simulate_WithSettings import *
import cvxpy as cp
from cvxopt import solvers
from cycler import cycler
from UUV_Simulation import *
from scipy.optimize import linprog
import random
Alpha=0.9#0.1
Alpha_SS=0.5#0.1#0.05
Alpha_EC=0.9#0.1#0.1
# Accuracy , Scanning Speed, Energy Consumption
Y_d = np.array([1.0, 3.1, 142.0])# np.array([1.0, 3.1, 142.0])# np.array([1.0, 3.1, 142.0])
Y_d_plot=[]
Error_Plot=[]
Sensors_Plot=[]
Y_d_plot.insert(len(Y_d_plot), [Y_d[0], Y_d[1], Y_d[2]])
kol=200
k_change_Setpoint_EC=50000
k_change_Setpoint_SS=1000 ###*** changed
k_Change_Setting=5000
k_Failure=2000000
k_DeleteGoal=1000000
k_optimal= 100
n=3
m=5
Us=[]
Ys=[]

W = np.array([1.e-10, 100.0 , 0.01])#np.array([1.e-10, 10000.0 , 0.01])#np.array([1.0, 10.0 , 0.1])
u_0=np.array([[0.1,0.1,0.3,0.3,0.2]])
u_1=np.array([[0.0,0.3,0.2,0.2,0.3]])
Us.insert(len(Us),u_0)
Us.insert(len(Us),u_1)
diff_u_1= u_1 -u_0      
obj= UUV_Sim()
y_0=obj.UUV_Run(u_0.reshape(m,1)).reshape(n,1)
obj= UUV_Sim()
y_1=obj.UUV_Run(u_1.reshape(m,1)).reshape(n,1)
yy=y_0.squeeze()
Ys.insert(len(Ys),[yy[0] * 100.0,yy[1] , yy[2]])    
yy=y_1.squeeze()
U_SS= u_1 + (Y_d[1] - yy[1]) * Alpha_SS
Y_SS= yy[1] #+ (Y_d[1] - yy[1]) * Alpha_SS
Y_EC = yy[2] #+ (Y_d[2] - yy[2]) * Alpha_EC
Ys.insert(len(Ys),[yy[0] * 100.0 ,yy[1] , yy[2]]) 
print("Inital _y0:",y_0.squeeze())
print("Inital _y1:",y_1.squeeze())
diff_y_1= y_1 - y_0  # Shape (3,1)
n2=np.linalg.norm(diff_u_1)
Unit=np.array([[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0,1.0]])
a=0.001

Phy= Unit

e_0= abs(Y_d.reshape(n,1) - y_0)    # Shape (3,1)
e_1=  abs(Y_d.reshape(n,1) - y_1)   # Shape (3,1)

for k in range(kol):

    if k==k_change_Setpoint_EC:
        
        Y_d = np.array([1.0, 2.7, 142.0])
    if k==k_change_Setpoint_SS:
        Y_d = np.array([1.0, 3.1, 78.0])#np.array([0.49, 3.1, 78.0])
        # Y_d = np.array([1.0, 3.1, 100.0])
        


    if n2 == 0.0:
        Coefficient_Phy=np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])
    else:
        Coefficient_Phy = diff_u_1 / n2

    Diff_Error=diff_y_1 - Phy.dot(diff_u_1.reshape(m,1))

    Phy_Update=Phy+np.squeeze(Coefficient_Phy* Diff_Error[:,None])

    
    if k== k_DeleteGoal :
        n = 2
        Y_d = np.array([ 3.6, 0.0])
        Y_d2 = np.array([1.0,3.6, 0.1])
        
        W=np.delete(W,0,0)
        
        y_1 = np.delete(y_1, 0, 0)
        
        Phy_Update = np.delete(Phy_Update, 0, 0)
        Phy = np.delete(Phy, 0, 0)

    ##############################
    ######### Controller
    #############################
    # Y_K=y_1
    # U_K=u_1.squeeze()
    # X_U_next = cp.Variable(5)#, complex=True)
    #
    # constraints = [
    #
    #
    #                  X_U_next[0] >=-1.0 , X_U_next[0] <=1.0,
    #                  X_U_next[1] >= -1.0, X_U_next[1] <= 1.0,
    #                  X_U_next[2] >= -1.0, X_U_next[2] <= 1.0,
    #                  X_U_next[3] >= -1.0, X_U_next[3] <= 1.0,
    #                  X_U_next[4] >= -1.0, X_U_next[4] <= 1.0,
    #
    #                (U_K[0] + X_U_next[0]) + (U_K[1] + X_U_next[1]) + (U_K[2] + X_U_next[2]) + (U_K[3] + X_U_next[3])+ (U_K[4] + X_U_next[4])== 1.0,
    #                 (U_K[0]+X_U_next[0]) >= 0.0, (U_K[0]+X_U_next[0]) <= 1.0,
    #                 (U_K[1]+X_U_next[1]) >= 0.0, (U_K[1]+X_U_next[1]) <= 1.0,
    #                 (U_K[2]+X_U_next[2]) >= 0.0, (U_K[2]+X_U_next[2]) <= 1.0,
    #                 (U_K[3]+X_U_next[3]) >= 0.0, (U_K[3]+X_U_next[3]) <= 1.0,
    #                 (U_K[4]+X_U_next[4]) >= 0.0, (U_K[4]+X_U_next[4]) <= 1.0,
    #                 (Y_K.squeeze() + Phy @ X_U_next)[1] >= 3.08,
    #                 (Y_K.squeeze() + Phy @ X_U_next)[1] <= 3.2,
    #                 (Y_K.squeeze() + Phy @ X_U_next)[2] >= 141.5,
    #                 (Y_K.squeeze() + Phy @ X_U_next)[2] <= 142.5,
    #
    #
    #                ]
    #
    # # if k>= k_change_Setpoint_SS:
    # #     constraints = [
    # #
    # #
    # #         X_U_next[0] >= -1.0, X_U_next[0] <= 1.0,
    # #         X_U_next[1] >= -1.0, X_U_next[1] <= 1.0,
    # #         X_U_next[2] >= -1.0, X_U_next[2] <= 1.0,
    # #         X_U_next[3] >= -1.0, X_U_next[3] <= 1.0,
    # #         X_U_next[4] >= -1.0, X_U_next[4] <= 1.0,
    # #
    # #         (U_K[0] + X_U_next[0]) + (U_K[1] + X_U_next[1]) + (U_K[2] + X_U_next[2]) + (U_K[3] + X_U_next[3]) + (
    # #                     U_K[4] + X_U_next[4]) == 1.0,
    # #         (U_K[0] + X_U_next[0]) >= 0.0, (U_K[0] + X_U_next[0]) <= 1.0,
    # #         (U_K[1] + X_U_next[1]) >= 0.0, (U_K[1] + X_U_next[1]) <= 1.0,
    # #         (U_K[2] + X_U_next[2]) >= 0.0, (U_K[2] + X_U_next[2]) <= 1.0,
    # #         (U_K[3] + X_U_next[3]) >= 0.0, (U_K[3] + X_U_next[3]) <= 1.0,
    # #         (U_K[4] + X_U_next[4]) >= 0.0, (U_K[4] + X_U_next[4]) <= 1.0,
    # #
    # #         (Y_K.squeeze() + Phy @ X_U_next)[0] >= 0.80,
    # #
    # #     ]
    #
    # if k >= k_Failure:
    #     constraints = [
    #
    #     # One Sensor is failed
    #     # X_U_next[2] == 0.0,
    #     X_U_next[0] >= -1.0, X_U_next[0] <= 1.0,
    #     X_U_next[1] >= -1.0, X_U_next[1] <= 1.0,
    #     X_U_next[2] >= -1.0, X_U_next[2] <= 1.0,
    #     X_U_next[3] >= -1.0, X_U_next[3] <= 1.0,
    #     X_U_next[4] >= -1.0, X_U_next[4] <= 1.0,
    #     ##############################
    #     # ###### Check SL Next State
    #     (U_K[2] + X_U_next[2])==0.0,
    #     (U_K[0] + X_U_next[0]) + (U_K[1] + X_U_next[1])+ (U_K[2] + X_U_next[2]) + (U_K[3] + X_U_next[3]) + (U_K[4] + X_U_next[4]) == 1.0,
    #     (U_K[0] + X_U_next[0]) >= 0.0, (U_K[0] + X_U_next[0]) <= 1.0,
    #     (U_K[1] + X_U_next[1]) >= 0.0, (U_K[1] + X_U_next[1]) <= 1.0,
    #     # (U_K[2] + X_U_next[2]) >= 0.0, (U_K[2] + X_U_next[2]) <= 1.0,
    #     (U_K[3] + X_U_next[3]) >= 0.0, (U_K[3] + X_U_next[3]) <= 1.0,
    #     (U_K[4] + X_U_next[4]) >= 0.0, (U_K[4] + X_U_next[4]) <= 1.0,
    # ]
    # Diff_Variable = Phy @ X_U_next
    # U_Diff_opt = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
    #
    # b_Controller = ((Y_d - np.reshape(Y_K, (1, n)))).reshape(n, )
    # # prob_Controller = cp.Problem(cp.Minimize(cp.norm(W  @ cp.square(b_Controller - Diff_Variable), 1)), constraints)
    # prob_Controller = cp.Problem(cp.Minimize((Y_K.squeeze() + Phy @ X_U_next)[0] * -1.0), constraints)
    # prob_Controller.solve(cp.settings.MOSEK)
    # print("status:", prob_Controller.status)
    # print("Cost Value:", prob_Controller.value)
    # if prob_Controller.status.find("infeasible") != -1:
    #     U_Diff_opt = np.array([[0.0], [0.0], [0.0], [0.0], [0.0]])
    #
    # else:
    #     U_Diff_opt = (X_U_next.value).reshape(m,1) #Shape 5,1
    #
    #
    # U_K_n_opt = U_K + U_Diff_opt.reshape(1, m)  #Shape 1,5
    ###################################
    Alpha_SS = 0.1#0.05#random.random()/10#np.random.rand()# 0.1  # 0.1#0.05
    Alpha_EC = 0.3#random.random()/10#np.random.rand()#0.1
    # U_SS1 = u_1 + (Y_d[1] - yy[1]) * Alpha_SS
    # U_SS2 = [np.clip(U_SS1.squeeze()[i], 0.0, 1.0) for i in range(m)]
    # U_SS = U_SS2 / sum(U_SS2)
    diff_U_SS = np.array(m * [(Y_d[1] - yy[1])* Alpha_SS]) # U_SS.squeeze() - u_1.squeeze()
    Y_SS = yy[1] + (Phy @ diff_U_SS.reshape(m, 1))[1]

    # U_EC1 = u_1 + (Y_d[2] - yy[2]) * Alpha_EC
    # U_EC2 = [np.clip(U_EC1.squeeze()[i], 0.0, 1.0) for i in range(m)]
    # U_EC = U_EC2 / sum(U_EC2)
    diff_U_EC = np.array(m * [(Y_d[2] - yy[2])* Alpha_EC] )#U_EC.squeeze() - u_1.squeeze()
    Y_EC = yy[2] + (Phy @ diff_U_EC.reshape(m, 1))[2]
###############################################################
    # Alpha_SS=np.random.rand()
    # Alpha_EC = np.random.rand()
    # Y_SS= yy[1] + (Y_d[1] - yy[1]) * Alpha_SS
    # Y_EC = yy[2] + (Y_d[2] - yy[2]) * Alpha_EC


#######################Optimization Simplex##############
    if k >= k_optimal:

        obj = [-97, -89, -83, -74, -49] #[170, 135, 118, 100, 78]#
        lhs_eq = [[1.0, 1.0, 1.0, 1.0, 1.0]]#[2.6, 3.6, 2.6, 3.0, 3.6], [170, 135, 118, 100, 78]]
        rhs_eq = [1]#Y_SS, Y_EC]
        bnd = [(0, 1), (0, 1), (0, 1), (0, 1), (0, 1)]

        optimization = linprog(c=obj,
                               # A_ub = lhs,
                               # b_ub = rhs,
                               bounds=bnd,
                               A_eq=lhs_eq,
                               b_eq=rhs_eq,
                               method='simplex')  # 'simplex' 'highs'

        vec = optimization.x
    else :
        obj = [-97, -89, -83, -74, -49]  # [170, 135, 118, 100, 78]#
        lhs_eq = [[1.0, 1.0, 1.0, 1.0, 1.0] ,[2.6, 3.6, 2.6, 3.0, 3.6], [170, 135, 118, 100, 78]]
        rhs_eq = [1, Y_SS, Y_EC]
        bnd = [(0, 1), (0, 1), (0, 1), (0, 1), (0, 1)]

        optimization = linprog(c=obj,
                               # A_ub = lhs,
                               # b_ub = rhs,
                               bounds=bnd,
                               A_eq=lhs_eq,
                               b_eq=rhs_eq,
                               method='simplex')  # 'simplex' 'highs'

        vec = optimization.x


    U_K_n_opt = vec
    Sensors= U_K_n_opt.squeeze()
    Sensors_Plot.insert(len(Sensors_Plot),[Sensors[0] * 100.0 ,Sensors[1] * 100.0 ,Sensors[2] * 100.0,Sensors[3] * 100.0 ,Sensors[4] * 100.0])

    if k >= k_DeleteGoal :
       y_0 = y_1  # shape 2,1
       obj = UUV_Sim()

       if (k >= k_Change_Setting):
           obj.EnergyCons_Settings = np.array([190, 135, 118, 100, 78])

       y1_exec= obj.UUV_Run(np.squeeze(U_K_n_opt.reshape(m,1)))
       yy = y1_exec.squeeze()
       y1_exec=np.delete(y1_exec,0)
       y_1=y1_exec.reshape(n,1)
       Ys.insert(len(Ys), [yy[0] * 100.0, yy[1], yy[2]])  # yy[1] *10
       Y_d_plot.insert(len(Y_d_plot), [Y_d2[0], Y_d2[1], Y_d2[2]])
       Error_Plot.insert(len(Error_Plot), [abs(Y_d2[0] - yy[0]), abs(Y_d2[1] - yy[1]), abs(Y_d2[2] - yy[2])])
       
    else:
        y_0 = y_1  # shape 3,1
        obj = UUV_Sim()

        if (k >= k_Change_Setting):
            obj.EnergyCons_Settings = np.array([190, 135, 118, 100, 78])

        y_1 = obj.UUV_Run(np.squeeze(U_K_n_opt.reshape(m, 1))).reshape(n, 1)
        yy = y_1.squeeze()
        # U_SS1 = u_1 + (Y_d[1] - yy[1]) * Alpha_SS
        # U_SS2 = [np.clip(U_SS1.squeeze()[i], 0.0, 1.0) for i in range(m)]
        # U_SS= U_SS2 / sum(U_SS2)
        # diff_U_SS= U_SS.squeeze() - u_1.squeeze()
        # Y_SS = yy[1] + (Phy @ diff_U_SS.reshape(m,1))[1]
        # # Y_SS= yy[1] + (Y_d[1] - yy[1]) * Alpha_SS
        # U_EC1 = u_1 + (Y_d[2] - yy[2]) * Alpha_EC
        # U_EC2 = [np.clip(U_EC1.squeeze()[i], 0.0, 1.0) for i in range(m)]
        # U_EC = U_EC2 / sum(U_EC2)
        # diff_U_EC =  U_EC.squeeze() - u_1.squeeze()
        # Y_EC = yy[2] + (Phy @ diff_U_EC.reshape(m,1))[2]
        # # Y_EC = yy[2] + (Y_d[2] - yy[2]) * Alpha_EC
        Ys.insert(len(Ys), [yy[0] * 100.0, yy[1], yy[2]])  # yy[1] *10
        Y_d_plot.insert(len(Y_d_plot), [Y_d[0], Y_d[1], Y_d[2]])
        Error_Plot.insert(len(Error_Plot), [abs(Y_d[0] - yy[0]), abs(Y_d[1] - yy[1]), abs(Y_d[2] - yy[2])])



    
###############################################
    diff_y_1= y_1 - y_0 #shape 3,1

    

    u_0=u_1.reshape(1,m) #shape 1,5
    u_1=U_K_n_opt.reshape(1,m) #shape 1,5
    Us.insert(len(Us),u_1)#np.round(u_1,1))
    diff_u_1= u_1 -u_0 #shape 1,5
    n2=np.linalg.norm(diff_u_1)
    

    Phy=Phy_Update

  

    print("Ys[Outputs] after iteration{}:{}".format(k, np.squeeze(y_1)))
    print("Us[Inputs] after iteration{}:{}".format(k, np.squeeze(np.round(u_1,5))))
    print(" Senesor Usange after iteration{}:{} ".format(k,np.round(Sensors,3)))

    



# Error_Plot
y1=plt.figure("Accuracy")

plt.plot(list(zip(*Ys))[0],linestyle='solid', color='blue' , label ='Actual accuracy')

plt.ylabel('%' ,loc='top' ,rotation=0)#,fontsize=18)
plt.xlabel('k', loc='right')#,fontsize=18)
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=1)  ## 'upper right'
plt.title("Accuracy" )#,fontsize=18, x=0.5 ,y=0.9)
plt.grid('on')
plt.ylim((0,100))

y1.savefig('results/UUV_OneOperationMode/Accuracy.png')

y2=plt.figure("Scanning Speed")

# plt.plot([i  for i in  list(list(zip(*Ys))[1])]  ,color='k' ,linestyle='solid', label ='Measured speed')
# plt.plot(list(zip(*Y_d_plot))[1],color='red', linestyle='dotted', label='Speed goal')
# plt.plot(list(zip(*Error_Plot))[1],linestyle='solid',color='blue', label='Speed error')
plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_optimal-1)]))[1],color='k' ,linestyle='solid', label ='Measured speed')
plt.plot(list(zip(*[Y_d_plot[ki] for ki in range(1,k_optimal-1)]))[1],color='red', linestyle='dotted', label='Speed goal')
plt.ylabel('m/s', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.5, -0.1),ncol=3)  
plt.title("Scanning Speed")
plt.grid('on')
plt.ylim((2.0,3.7))
y2.savefig('results/UUV_OneOperationMode/Scanning Speed.png')

y3=plt.figure("Energy Consumption")
# plt.plot([i  for i in  list(list(zip(*Ys))[2])], color='k' ,linestyle='solid', label='Measured energy') #### Optimal
# plt.plot([i  for i in  list(list(zip(*Y_d_plot))[2])],color='red', linestyle='dotted', label='Energy goal')
# plt.plot([i  for i in  list(list(zip(*Error_Plot))[2])],linestyle='solid' ,color='blue', label='Energy error')
plt.plot(list(zip(*[Ys[ki] for ki in range(1,k_optimal+1)]))[2], color='k' ,linestyle='solid', label='Measured energy')
plt.plot(list(zip(*[Y_d_plot[ki] for ki in range(1,k_optimal)]))[2],color='red', linestyle='dotted', label='Energy goal')
plt.plot(range(k_optimal,kol) , list(zip(*[Ys[ki] for ki in range(k_optimal,kol)]))[2],  color='m' ,linestyle='solid', label='Optimal energy' )


plt.ylabel('J/s', loc='top',rotation=0)

plt.xlabel('k', loc='right')
plt.legend(loc='center', bbox_to_anchor=(0.45, -0.1),ncol=3)
plt.title("Energy Consumption")
plt.grid('on')
plt.ylim((70,180))
y3.savefig('results/UUV_OneOperationMode/Energy Consumption.png')

Sense=plt.figure("Sensor Usange")
Sense.tight_layout()
plt.plot(list(zip(*Sensors_Plot))[0],color='red',linestyle='solid', label='Sensor 1')
plt.plot(list(zip(*Sensors_Plot))[1],color='blue',linestyle= 'dashed', label='Sensor 2')
plt.plot(list(zip(*Sensors_Plot))[2],color='lime',linestyle= (0,(10,2,2,2,2,2)) , label='Sensor 3')
plt.plot(list(zip(*Sensors_Plot))[3],color='brown',linestyle= (0,(2,2,10,2,10,2)) , label='Sensor 4')
plt.plot(list(zip(*Sensors_Plot))[4],color='magenta',linestyle= 'dotted', label='Sensor 5')
plt.ylabel('%',loc='top',rotation=0) 
plt.xlabel('k', loc='right') 
plt.legend(loc='center', bbox_to_anchor=(0.45, -0.10),ncol=5, fontsize=8)
plt.title("Sensor usage (time fractions)" )
plt.grid('on')
plt.ylim((0,100))
Sense.savefig('results/UUV_OneOperationMode/Sensor Usange.png')

plt.show()



