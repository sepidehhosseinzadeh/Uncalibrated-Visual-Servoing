#!/usr/bin/env python
import rospy
import ipdb
import numpy

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from visual_servoing.msg import Error
from wam_srvs.srv import Hold
from wam_srvs.srv import JointMove
from time import sleep

class ErrorHandler(object):
    """Subscribes to the image_error topic and keeps track of the image error
    """
    def _image_error_cb(self, data):
        """Callback function for the /image_error topic

            image_error is an n-tuple with the defined image_errors

            NOTE:
                Do not call manually
        """
        self.image_error = data.error

    def __init__(self):
        super(ErrorHandler, self).__init__()
        self.image_error_sub = rospy.Subscriber("image_error", Error, self._image_error_cb)
        self.image_error = None

class WAM(object):
    """ Wrapper for the WAM services and messages
    """

    def tool_pose_cb(self, data):
        """ Callback function for the /wam/pose topic.
            The last tool pose is saved in self.last_tool_pose
            This is an object with two members position and orientationi where
            the orientation is a quaternion:

            eg.
            xyz_position = [wam.last_tool_pose.position.x,
                            wam.last_tool_pose.position.y,
                            wam.last_tool_pose.position.z
                           ]
            pose_quaternion = [wam.last_tool_pose.orientatoin.x,
                               wam.last_tool_pose.orientatoin.y,
                               wam.last_tool_pose.orientatoin.z,
                               wam.last_tool_pose.orientatoin.w
                              ]

            NOTE:
                Do not call manually
        """
        self.last_tool_pose = data.pose

    def joint_pose_cb(self, data):
        """ Callback function for the /wam/joint_states topic.

            last_joint_pose is a 4-tuple with radian angles of each joint

            NOTE:
                Do not call manually
        """
        self.last_joint_pose = data.position

    def go_home(self):
        """ Sends the robot safely to it's home position
        """
        self.go_home_srv()

    def joint_move(self, joints):
        """ Given a list of 4 angles in rads it will command the robot to move
            to the given joint angles.

            eg. joint_move([0,0,0,1.57])

            Args:
                joints (List[double]): Joint angles in radians for the robot to
                                      move to
        """
        if len(joints) != 4:
            rospy.logwarn("Invalid joint move target: " + str(joints))
            return
        self.joint_move_srv.call(joints)

    def hold(self, hold):
        """ Allows for the robot to be set in a hold state or in gravity
            compensation. When in hold, the robot can only be moved through
            commands. When in gravity compensation you can manually move the
            arm around.

            Args:
                hold (bool): Whether the robot will be in hold or not
        """
        self.hold_srv.call(hold)

    def __init__(self):
        """ Creates the WAM object and subscribes to the approriate services and topics
        """
        super(WAM, self).__init__()
        self.last_tool_pose = None
        self.last_joint_pose = None

        robot_ns = rospy.get_param('~robot_namespace', 'slax')
        self.tool_pose_sub = rospy.Subscriber(robot_ns + "/wam/pose", PoseStamped, self.tool_pose_cb)
        self.joint_pose_sub = rospy.Subscriber(robot_ns + "/wam/joint_states", JointState, self.joint_pose_cb)

        self.go_home_srv = rospy.ServiceProxy(robot_ns + "/wam/go_home", Empty)
        self.joint_move_srv = rospy.ServiceProxy(robot_ns + "/wam/joint_move", JointMove)
        self.hold_srv = rospy.ServiceProxy(robot_ns + "/wam/hold_joint_pos", Hold)

def inv_kin(wam, target):
    """ Given a target (X,Y,Z) return the joint angles required to reach the
        target

        Args:
            wam (WAM) : wam proxy object
            target (List[double]) : The target [X,Y,Z] position of the end
                                    effector.
        Returns:
            angles (List[double]) : Angles each joint should assume to reach
                                    the desired target.

    """
    # TODO: Your code HERE!    
    # First move the arm to an initial position
    # Initialize thetas to avoid singular J
    
    thetas= [-0.15255804169043624, 1.0113141658044644, 0.20967885825310853, 1.8937845037997065]
    wam.joint_move(thetas)
    sleep(10)
    error= 10000
    print('Thetas : ', thetas)
    jacobian= get_jacobian(wam, 0.15)
    print(thetas) 
    thetas=list(wam.last_joint_pose)
    x=raw_input('Press any key')
    niters= 0
    maxIters = 100
    errVect = [0.0]*maxIters
    lamda = 0.05 # rate in the Control law
    alpha = 0.025 # rate in the Broyden update
    t= numpy.matrix(target).getT()
    #transformation_matrix= numpy.matrix('')
    #t= numpy.matrix(target).getT()*transformation_matrix
    #t= numpy.delete(t, 3)
    print('target: ', t)
    # initial position
    f2= numpy.matrix([wam.last_tool_pose.position.x, wam.last_tool_pose.position.y, wam.last_tool_pose.position.z]).getT() - t
    minDelta= numpy.pi/360.0
    while(error> 0.003 and niters<maxIters):
        f1 = f2
        f= -1*f1
        s=numpy.matrix('0.0; 0.0; 0.0; 0.0') # Delta Theta 
        try:
            # Moore penrose pseudoinverse
            #new_lamda=0.05+lamda*(1.0/(maxIters-niters+1))
            new_lamda=0.15
            print('new Lamda is ', new_lamda)
            s= new_lamda*numpy.linalg.inv(jacobian.getT()*jacobian)*jacobian.getT()* f
            print('s is', s)
        except:
            print('Singular Matrix :D')
        #print('Thetas Before: ', thetas)
        #print('dTheta Before: ', s)
        #if (numpy.linalg.norm(s, 1) < minDelta):
        #    break
                
        check_delta(thetas, numpy.array(s).reshape(-1,).tolist())
        prev_joint_pose= wam.last_joint_pose
        print('Thetas After: ', thetas)
        print('dTheta After: ', s)
        #x=raw_input('Press any key')
        wam.joint_move(thetas)
        sleep(1.0)
        f2= numpy.matrix([wam.last_tool_pose.position.x, wam.last_tool_pose.position.y, wam.last_tool_pose.position.z]).getT() - t
        deltaY= f2- f1
        error = numpy.sum(numpy.absolute(f2))
        errVect[niters]=error
        ipdb.set_trace()
        #print('Jacobian before the update', jacobian)
        ## Simulate Quazi Newton with one Jacobian 
        #jacobian = broyden_update(wam, [], jacobian, alpha, prev_joint_pose  , deltaY)
        #print('Jacobian after the update', jacobian)
        print('Error Vector after Update : ', f2)
        print('Error: ', error) 
        niters+=1
        #x=raw_input('Press any key')
    import matplotlib.pyplot as plt
    plt.plot(errVect)
    plt.ylabel('L1 error')
    plt.show(block=False)
    return thetas

def check_delta(theta, step):
    max_theta= numpy.pi/2
    max_delta = numpy.pi/10 # 18 degrees
    for i in range(0, 4):
        # limit the delta
        step[i] = numpy.sign(step[i])*max_delta if abs(step[i]) > max_delta else step[i]
        theta[i] += step[i]
        # limit the theta
        theta[i]= numpy.sign(theta[i]) * (abs(theta[i]) % (2*numpy.pi))
        theta[i] = max_theta*numpy.sign(theta[i]) if abs(theta[i]) > max_theta else theta[i]

        
def get_jacobian(wam, delta):
    """ This function should acquire the Jacobian by performing small movements
        of the arm.

        Args:
            wam (WAM) : wam proxy object
            delta (double) : The step size the robot should take when
                             calculating the Jacobian
        Returns:
            jacobian (numpy.matrix) : Numpy matrix of shape (k x d) where k is
                                      the number of visual features, and d the
                                      number of DOF being controlled

    """
    # TODO: Your code HERE!
    J = numpy.matrix('0.0 0.0 0.0 0.0;0.0 0.0 0.0 0.0;0.0 0.0 0.0 0.0')
    thetas = wam.last_joint_pose
    print(wam.last_tool_pose)
    pos= wam.last_tool_pose.position
    for i in range(0, 4):
        deltaT= [0,0,0,0]
        deltaT[i]= delta
        thetasNew= [x + y for x, y in zip(thetas, deltaT)]
        wam.joint_move(thetasNew)
        sleep(2)
        print(wam.last_tool_pose)
        pos1= wam.last_tool_pose.position
        dx= (pos1.x- pos.x)/(delta)
        dy= (pos1.y- pos.y)/(delta)
        dz= (pos1.z- pos.z)/(delta)
        # Set Jacobian entries
        J[0,i]=dx
        J[1,i]=dy
        J[2,i]=dz
        wam.joint_move(thetas)
        sleep(2)
    print('Last thetas:', wam.last_joint_pose)
    print J
    return J

def step(wam, error, jacobian, lambda_step):
    """ This function should perform one step of the Visual Servoing control
        law every time it is called

        Args:
            wam (WAM) : wam proxy object
            error (ErrorHandler) : error handler object
            jacobian (numpy.matrix) : Numpy matrix of shape (k x d) where k is
                                      the number of visual features, and d the
                                      number of DOF being controlled
            lambda_step (double) : The step size the robot should take when
                                   following the control law
    """
    # TODO: Your code HERE!
    pass

def broyden_update(wam, error, jacobian, alpha, prev_joint_pose, prev_image_error):
    """ Perform a broyden update of the Jacobian

        Args:
            wam (WAM) : wam proxy object
            error (ErrorHandler) : error handler object
            jacobian (numpy.matrix) : Numpy matrix of shape (k x d) where k is
                                      the number of visual features, and d the
                                      number of DOF being controlled
            alpha (double) : learning rate for the Broyden update
            prev_joint_pose (tuple(double)) : joint pose when the previous
                                              Jacobian update occured
            prev_image_error (tuple(double)) : image error when the previous
                                               Jacobian update occured (deltaY)
    """
    # TODO: Your code HERE!
    x2= numpy.matrix(wam.last_joint_pose).getT()
    x1= numpy.matrix(prev_joint_pose).getT()
    deltaX= x2 - x1
    #print('We have reached here')
    eps = numpy.pi/360
    #if ((deltaX.getT()*deltaX)!=0):
    if (deltaX >= eps).any():
        jacobian= jacobian + alpha*((prev_image_error-jacobian*deltaX)*deltaX.getT())/ (deltaX.getT()*deltaX)
    return jacobian
    #t= numpy.matrix(target)*transformation_inv
    #t= numpy.matrix(target)*transformation_inv

if __name__ == '__main__':
    rospy.init_node("UVSNode")
    wam = WAM()
    error = ErrorHandler()

    ipdb.set_trace()

    rospy.spin()
    print "Shutting down UVSNode"
