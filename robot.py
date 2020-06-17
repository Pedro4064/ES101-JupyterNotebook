import vectors
import matplotlib.pyplot as plt
import numpy as np

class robotic_arm():

    def __init__(self):
        
        ######## Create variables for this class ########
        
        # The origin point for the robotic arm  
        self.origin_point = vectors.point(0,0)

        # the vector representing the rotating arm at the default position
        self.arm_vector = vectors.vector2(x=500,y=0)

        # The vector that represents the retracting arm
        self.retracting_arm_vector = vectors.vector2(500,0)
        # The percentage of the retracting arm that is out 
        self.retracting_arm_vector_percentage = 1.0

        # The target point used in the reverse kinematics
        self.target = vectors.point(0,0)

    def reset_rotating_arm(self):

        # the vector representing the rotating arm at the default position
        self.arm_vector = vectors.vector2(x=500,y=0)

    def reset_retracting_arm(self):

        # The vector that represents the retracting arm
        self.retracting_arm_vector = vectors.vector2(500,0)

    def set_target(self, target_point):

        self.target = target_point

    def calculate_movemet_to_target(self):

        # First determine the vector from the origin point of the robot to the target point 
        v = self.origin_point.vector_from_points(self.target)

        # If the norm of the v vector is smaller than 500, it's outside the workspace since the smallest
        # vector possible is when the retracting arm = 0
        if v.norm() < 500:
            raise KeyError('[ERROR] Range error: Point outside workspace')

        # Check to see if the angle between the v target vector and the x axis is outside the arms range (0<=angle<=90)
        reference_angle = v.angle()
        if reference_angle > 90 or reference_angle<0:
            raise KeyError('[ERROR] Range Error: The angle desired %s is outsie the range of motion this robot is capable' %(str(reference_angle)))
       
        # Calculate the angle which the arm will have to rotate from its current position to go to the target position
        target_angle = v.angle(self.arm_vector)

        # Rotate the arm vector that amount 
        self.arm_vector = self.arm_vector.rotate_vector(target_angle)

        # subtract the arm vector from the target v vector, which will result in how much the retracting arm will need to move 
        self.retracting_arm_vector = v - self.arm_vector

        # The max length of the retracting is 0,5m (aka 500mm)so if the nor of the vector is gratter than that, raise an error
        if self.retracting_arm_vector.norm() > 500:
            raise KeyError('[ERROR] Range error: Point outside workspace')
        
        print('The angle of movement is:',target_angle)
        print('The vector representing the rotating arm is:', self.arm_vector)
        print('The vector representing the retracting arm is:', self.retracting_arm_vector)

    def rotate_robotic_arm(self,angle):

        # rotate the robotic arm 
        self.arm_vector = self.arm_vector.rotate_vector(angle - self.arm_vector.angle())
        
        # also 'rotate' the retracting arm 
        self.retracting_arm_vector = self.retracting_arm_vector.rotate_vector(angle)

    def extand_arm(self, percentage):
        
        self.retracting_arm_vector_percentage = percentage
        self.retracting_arm_vector = self.arm_vector.multiplication_by_scalar(percentage)

    def plot_robotic_arm(self):
        
        # Create the subplots
        fig,ax = plt.subplots()

        # create the vector representation of the rotating robotic arm
        ax.quiver(self.origin_point.x,self.origin_point.y, self.arm_vector.x , self.arm_vector.y,color='orange' , units='xy' ,scale=1)

        # Create the vector that will represent the retracting robot arm 
        plt.quiver(self.arm_vector.x,self.arm_vector.y, self.retracting_arm_vector.x, self.retracting_arm_vector.y,color='b', units='xy' ,scale=1)

        # Equalize the axis and vectors 
        ax.set_aspect('equal')

        # set the grids 
        plt.grid()

        # set the limits in the graph 
        plt.xlim(0,1000)
        plt.ylim(0,1000)

        # set the labels 
        plt.xlabel('X component (mm)')
        plt.ylabel('Y component (mm)')

        plt.title('Robotic Arm')

        # show the plot
        plt.show()

    def plot_workspace(self):

        x_coordinates = []
        y_coordinates = []

        # Iterate through all the possible angles for the rotating robotic arm 
        for angle in np.arange(0,90.01,0.01):

            # move the robotic arm 
            self.rotate_robotic_arm(angle)

            # Iterate through all the possible lengths of the extending arm 
            for percentage_of_arm in np.arange(0,1.001,0.001):

                # update the arm lenght 
                self.extand_arm(percentage_of_arm)

                # Calculate the x coordinate of the tip of the arm
                x_coordinates.append(self.origin_point.x+self.arm_vector.x+self.retracting_arm_vector.x)

                # Calculate the y cooridnate of the tip of the arm 
                y_coordinates.append(self.origin_point.y+self.arm_vector.y+self.retracting_arm_vector.y)

                # reset the lenght of the arm 
                self.reset_retracting_arm()


            # set the robotic arm back to the original position 
            self.reset_rotating_arm()

        # Plot all the poinst 
        plt.plot(x_coordinates,y_coordinates,'o',color = ('black'))
        plt.title("Robot's Workspace")
        plt.show()


if __name__ == '__main__':

    robot = robotic_arm()
    # robot.plot_workspace()
    # target = vectors.point(0,1000)
    # robot.set_target(target_point=target)
    robot.rotate_robotic_arm(45)
    # # robot.extand_arm(0.5)
    # robot.calculate_movemet_to_target()
    robot.plot_robotic_arm()