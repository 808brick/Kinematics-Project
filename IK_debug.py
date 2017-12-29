from sympy import pi, cos, sin, symbols, simplify, sqrt, atan2
from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    class Kuka_KR210:

        #symbols used


        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #angles of revolute joint
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        r, p, y = symbols("r p y")

        #DH Parameter Table
        s = {alpha0: 0,      a0: 0,     d1: 0.75,
             alpha1: -pi/2.,  a1: 0.35,  d2: 0,       q2: q2-pi/2.,
             alpha2: 0,      a2: 1.25,  d3: 0,
             alpha3: -pi/2.,  a3: -0.054,d4: 1.5,
             alpha4: pi/2.,   a4: 0,     d5: 0,
             alpha5: -pi/2.,  a5: 0,     d6: 0,
             alpha6: 0,      a6: 0,     d7: 0.303,  q7: 0}

        #known Transform Matricies for Kuka_KR210 with given DH Parameter Table
        T0_1 = Matrix([[cos(q1), -sin(q1), 0, 0], [sin(q1), cos(q1), 0, 0], [0, 0, 1, 0.750000000000000], [0, 0, 0, 1]])

        T1_2 = Matrix([[cos(q2 - 0.5*pi), -sin(q2 - 0.5*pi), 0, 0.350000000000000], [0, 0, 1, 0], [-sin(q2 - 0.5*pi), -cos(q2 - 0.5*pi), 0, 0], [0, 0, 0, 1]])

        T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25000000000000], [sin(q3), cos(q3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        T3_4 = Matrix([[cos(q4), -sin(q4), 0, -0.0540000000000000], [0, 0, 1, 1.50000000000000], [-sin(q4), -cos(q4), 0, 0], [0, 0, 0, 1]])

        T4_5 = Matrix([[cos(q5), -sin(q5), 0, 0], [0, 0, -1, 0], [sin(q5), cos(q5), 0, 0], [0, 0, 0, 1]])

        T5_6 = Matrix([[cos(q6), -sin(q6), 0, 0], [0, 0, 1, 0], [-sin(q6), -cos(q6), 0, 0], [0, 0, 0, 1]])

        T6_G = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.303000000000000], [0, 0, 0, 1]])

        T0_2 = Matrix([[sin(q2)*cos(q1), cos(q1)*cos(q2), -sin(q1), 0.35*cos(q1)], [sin(q1)*sin(q2), sin(q1)*cos(q2), cos(q1), 0.35*sin(q1)], [cos(q2), -sin(q2), 0, 0.750000000000000], [0, 0, 0, 1]])

        T0_3 = Matrix([[sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1), (1.25*sin(q2) + 0.35)*cos(q1)], [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3), cos(q1), (1.25*sin(q2) + 0.35)*sin(q1)], [cos(q2 + q3), -sin(q2 + q3), 0, 1.25*cos(q2) + 0.75], [0, 0, 0, 1]])

        T0_4 = Matrix([[sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4), sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1), -sin(q1)*sin(q4)*sin(q2 + q3) - cos(q1)*cos(q4), sin(q1)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [cos(q4)*cos(q2 + q3), -sin(q4)*cos(q2 + q3), -sin(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_5 = Matrix([[(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-sin(q5)*sin(q2 + q3) + cos(q4)*cos(q5)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), sin(q4)*cos(q2 + q3), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_6 = Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        T0_G =  Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)], [((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)], [-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75], [0, 0, 0, 1]])

        R_x = Matrix([  [1,         0,          0],
                        [0,         cos(r),     -sin(r)],
                        [0,        sin(r),      cos(r)]])

        R_y = Matrix([  [cos(p),    0,      sin(p)],
                        [0,         1,      0],
                        [-sin(p),   0,      cos(p)]])



        R_z = Matrix([  [cos(y),    -sin(y),    0],
                        [sin(y),    cos(y),     0],
                        [0,         0,          1]])


        R_G = Matrix([[cos(p)*cos(y), sin(p)*sin(r)*cos(y) - sin(y)*cos(r), sin(p)*cos(r)*cos(y) + sin(r)*sin(y)], [sin(y)*cos(p), sin(p)*sin(r)*sin(y) + cos(r)*cos(y), sin(p)*sin(y)*cos(r) - sin(r)*cos(y)], [-sin(p), sin(r)*cos(p), cos(p)*cos(r)]])

    Kuka = Kuka_KR210()

    ########################################################################################
    ##

    # All Commented out print() statements can be uncommented for debugging purposes

    ## Insert IK code here!

    #Create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #angles of revolute joint
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    # print('Sybols Loaded')

    # # Retrieve Modified DH parameters from Kuka_KR210 Class
    s = Kuka.s
    # print('DH Parameter Table Loaded')

    #If new transform Matricies needed, call transform_matrix function with given q,alpha,a,d parameters and then substitute values with DH Paramter Table
    def transform_matrix(q, alpha, a, d):
        T_Matrix = Matrix([ [cos(q),               -sin(q1),               0,              a],
                        [sin(q1)*cos(alpha),   cos(q1)*cos(alpha),    -sin(alpha),   -sin(alpha)*d],
                        [sin(q1)*sin(alpha),   cos(q1)*sin(alpha),    cos(alpha),    cos(alpha)*d],
                        [0,                     0,                      0,              1]])
        return T_Matrix

    ##Example of Multiplying out Matricies
    # T0_2 = simplify(T0_1 * T1_2)
    # T0_3 = simplify(T0_2 * T2_3)
    # T0_4 = simplify(T0_3 * T3_4)
    # T0_5 = simplify(T0_4 * T4_5)
    # T0_6 = simplify(T0_5 * T5_6)
    # T0_G = simplify(T0_6 * T6_G)
    # # print('T0_G Matrix Loaded')

    #Assign Matrix Values from Kuka_KR210 Class
    T0_1 = Kuka.T0_1
    T1_2 = Kuka.T1_2
    T2_3 = Kuka.T2_3
    T3_4 = Kuka.T3_4
    T4_5 = Kuka.T4_5
    T5_6 = Kuka.T5_6
    T6_G = Kuka.T6_G
    T0_2 = Kuka.T0_2
    T0_3 = Kuka.T0_3
    T0_4 = Kuka.T0_4
    T0_5 = Kuka.T0_5
    T0_6 = Kuka.T0_6
    T0_G = Kuka.T0_G
    # print('All Matrixes and Transform Matrixes Loaded %04.4f seconds' % (time()-start_time))


    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y, req.poses[x].orientation.z, req.poses[x].orientation.w])

    r, p, y = symbols("r p y")

    #Define Rotation Matrixes using the precalculated ones from Kuka_KR210 Class
    R_x = Kuka.R_x
    R_y = Kuka.R_y
    R_z = Kuka.R_z
    R_G = Kuka.R_G
    # print('Rotations Matrixes Loaded %04.4f seconds' % (time()-start_time))

    #Account for Error in Gripper Rotation
    R_Err = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
    R_G = R_G * R_Err
    R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw}) #substitute symbols in R_G

    #Gripper Position
    G_pos = Matrix([ [px], [py], [pz]]) #gripper position

    #Wrist Position
    W_pos = G_pos - 0.303 * R_G[:,2] # wrist position
    # print('Wrist position Loaded %04.4f seconds' % (time()-start_time))

    #Calculate angles of revolute joints needed to reach desired Position
    theta1 = atan2(W_pos[1], W_pos[0]) #arc tan of wrist y,x position
    # print("Theta1:")
    # print(theta1)

    #Needed vars to solve for theta 2 & 3, using triangle and trig identities
    side1 = 1.501
    side2 = sqrt(pow((sqrt(pow(W_pos[0], 2) + pow(W_pos[1], 2))- 0.35) ,2) + pow((W_pos[2] - 0.75), 2))
    side3 = 1.25

    angle1 = acos((pow(side2, 2) + pow(side3, 2) - pow(side1, 2)) / (2*side3*side2))
    # print("angle1:")
    # print(angle1)

    angle2 = acos((pow(side1, 2) + pow(side3, 2) - pow(side2, 2)) / (2*side3*side1))
    # print("angle2:")
    # print(angle2)

    angle3 = acos((pow(side1, 2) + pow(side2, 2) - pow(side3, 2)) / (2*side1*side2))
    # print("angle3:")
    # print(angle3)

    theta2 = (pi/2) - angle1 - atan2(W_pos[2] - 0.75, (sqrt(pow(W_pos[0], 2) + pow(W_pos[1], 2))- 0.35))
    # print("Theta2:")
    # print(theta2)

    theta3 = (pi/2) - (angle2+0.036)
    # print("Theta3:")
    # print(theta3)

    #Needed vars to solve theata 4,5, & 6
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    R3_6 = R0_3.transform() *  R_G  #take inverse of R0_3 and multiply by Rotation of gripper

    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
    # print("Theta4:")
    # print(theta4)

    theta5 = atan2(sqrt(pow(R3_6[0,2], 2) + pow(R3_6[2,2], 2)), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])
    # print('All Thetas Loaded %04.4f seconds' % (time()-start_time))

    FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    # print('FK Loaded %04.4f seconds' % (time()-start_time))

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [W_pos[0],W_pos[1],W_pos[2]] # <--- Load your calculated WC values in this array
    print('Wrist Center Matrix Loaded')
    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    print('Gripper position Loaded')
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
