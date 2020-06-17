import math 
import numpy as np

class vector2():

    # The constructor for the class, it may take the x and y elements 
    # as parameters. The defaults are 0.0
    def __init__(self,x = 0.0, y=0.0):
        self.x = x
        self.y = y

    # sets the x and y components 
    def set_coordinates(self,x:float,y:float):
        self.x = x
        self.y = y

    # Calculates the norm of the vector
    def norm(self):
        vector_norm = math.sqrt(self.x**2 + self.y**2)
        return vector_norm

    # Calculates the angle between two vectors, if no other vector is passed
    # as an argument the calculations are done with the x vector (1,0)
    def angle(self, other_vector = None):
        # If the other vector was not passed as paramater, the default is the 'x' (1,0)
        if other_vector == None:
            other_vector = vector2(1,0) 
        
        # Since we know that the scalar product of two vectos equal the 
        # product between its norms and the angle between them:
        product = self * other_vector
        angle_cos = product / (self.norm() * other_vector.norm())
        angle = math.acos(angle_cos)

        # convert to degrees and returnt he value
        angle = math.degrees(angle)
        return angle

    # Multiply the vector by a scalar 
    def multiplication_by_scalar(self, scalar:float):
        return vector2(self.x*scalar,self.y*scalar)

    # Returns the unitary vector that is parallel to this one
    def unitary_vector(self):

        # We can find the unitary vector by deviding the vecotr by its norm
        unitary_vector = self.multiplication_by_scalar(1/self.norm())
        return unitary_vector

    # Returns a vector2 that is the origiginal vector but rotate ∂˚
    def rotate_vector(self, angle):

        # convert the angle to radians 
        angle = math.radians(angle)
        # Create the rotation matrix
        r_matrix = np.array([[math.cos(angle).__round__(5),-1*math.sin(angle).__round__(5)],
                             [math.sin(angle).__round__(5), math.cos(angle).__round__(5)]])
        
        # Create an array with the current vector2 componets
        arm_vector = np.array([[self.x],
                               [self.y]])

        # Multiply them 
        resulting_vecotr = np.matmul(r_matrix,arm_vector)

        # Return a vector 2 
        return vector2(resulting_vecotr[0], resulting_vecotr[1])

    # Operator overload for the sum of 2 vectors
    def __add__(self,other_vector):
        return vector2(x = self.x+other_vector.x, y = self.y+other_vector.y)

    # Operator overload for subtracting one vector from another
    def __sub__(self, other_vector):
        return vector2(x = self.x - other_vector.x, y = self.y - other_vector.y)

    # Gets the scalar product between two vectors 
    def __mul__(self, other_vector):
        scalar = (self.x * other_vector.x) + (self.y*other_vector.y)
        return scalar

    # The operator overload to represent each object through a string 
    def __str__(self):
        return "[{x},{y}]".format(x= self.x, y = self.y)

class point():

    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

    def set_coordinates(self,x,y):
        self.x = x
        self.y = y 

    # Generates a vector2 from 2 poinst(from self to another)
    def vector_from_points(self, other_point):

        # other - self 
        return vector2(x = other_point.x - self.x, y = other_point.y - self.y)

if __name__ == '__main__':

    p = point(0,0)
    q = point(10,0)

    v = p.vector_from_points(q)
    print(v)
    print(v.rotate_vector(180))