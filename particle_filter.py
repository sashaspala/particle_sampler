import numpy as np
import scipy
import create2
import time
import odometry


class ParticleFilter:

    def __init__(self):
        self.time = time
        self.locationX = 0
        self.locationY = 0
        self.orientation = 0
        self.odometry = odometry.Odometry()
        self.distance = 0

        self.probSensorGivenLoc = 0
        self.numParticles = 100
        self.probLoc = 1/self.numParticles
        self.sumProbs = self.probLoc
    #single particle first

    def movement(self, value, create):
        state = create.update()
        if value == 0: #forward
            create.drive_direct(100,100)
            self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            self.locationX = self.odometry.x
            self.locationY = self.odometry.y
            self.orientation = self.odometry.theta
            self.time.sleep(.5)
        elif value == 1: #turn left
            create.drive_direct(100,-100)
            self.time.sleep(.1)
        else: #turn right
            create.drive_direct(-100,100)
            self.time.sleep(.1)

    def sensing(self, create, sonar):
        distanceArr = []
        for x in range(0,self.numParticles):
            distanceArr[x] = sonar.get_distance()
        mean = np.average(distanceArr)
        std = np.std(distanceArr)
        self.probSensorGivenLoc = scipy.stats.norm(mean, std).pdf

    def estimation(self):
        #step 1: create 100 randomly distributed particles

        particleX = np.random.randint(0, 100, self.numParticles)
        particleY = np.random.randint(0, 100, self.numParticles)
        particleTheta = np.random.radint(0,180,self.numParticles)

        #getting sensor readings for each of the random robots
        particleSense = []
        for reading in (0, self.numParticles):
            particleSense = self.map.closest_distance((particleX[reading], particleY[reading]), particleTheta[reading])

        mean = np.average(particleSense)
        std = np.std(particleSense)

        #getting P(robot@location x | sensor reading)
        N = 1/sum(self.sumProbs)
        probability = (self.probSensorGivenLoc * self.probLoc) * N

        #now that we have P(virtual robot), kill off useless ones
        for value in particleSense:

            #TO DO: compare probabilities of particle sensor values to probability defined above (line 65)

        #updating probabilities
        self.probLoc = probability
        self.sumProbs += self.sumProbs
        #if particle is within 1 std deviation of mean value,
