import lab10_map
import math
import particle_filter
import create2
import odometry

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab10_map.Map("lab10.map")
        self.pf = particle_filter.ParticleFilter()
        self.odometry = odometry.Odometry()

    def run(self):
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        #self.virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
       # data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        #self.virtual_create.set_point_cloud(data)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        #print(self.map.closest_distance((0.5,0.5), math.pi))

        self.create.start()
        self.create.safe()
        self.create.start_stream([create2.Sensor.LeftEncoderCounts, create2.Sensor.RightEncoderCounts])

        #turning so we actually get a value!!
        self.create.drive_direct(-100,100)
        self.time.sleep(1.9)
        self.create.drive_direct(0,0)

        self.pf.senseOnce(self.create, self.sonar)
        # This is an example on how to detect that a button was pressed in V-REP
        while True:
            movement = False

            b = self.virtual_create.get_last_button()
            state = self.create.update()
            if b == self.virtual_create.Button.MoveForward:
                self.create.drive_direct(100,100)
                self.time.sleep(1)
                self.create.drive_direct(0,0)
                movement = True
                print("Forward pressed!")
            elif b == self.virtual_create.Button.TurnLeft:
                self.create.drive_direct(100,-100)
                self.time.sleep(1.9)
                self.create.drive_direct(0,0)
                movement = True
                print("Turn Left pressed!")
            elif b == self.virtual_create.Button.TurnRight:
                self.create.drive_direct(-100,100)
                self.time.sleep(1.9)
                self.create.drive_direct(0,0)
                movement = True
                print("Turn Right pressed!")
            elif b == self.virtual_create.Button.Sense:
                self.pf.sensing(self.create, self.sonar, self.virtual_create)
                print("Sense pressed!")

            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            self.locationX = self.odometry.x
            self.locationY = self.odometry.y
            self.orientation = self.odometry.theta
            self.pf.movement(self.locationX, self.locationY, self.orientation)
            if(movement):
               self.pf.sensing(self.create, self.sonar, self.virtual_create)
            self.time.sleep(0.01)
