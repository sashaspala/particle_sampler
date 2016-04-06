import numpy as np
from scipy import stats, misc
import time
import lab10_map
import math
import factory


class ParticleFilter:

    def __init__(self):
        self.time = time
        self.locationX = 0
        self.locationY = 0
        self.orientation = 0

        self.distance = 0

        #self.probSensorGivenLoc = 0
        self.numParticles = 100
        self.probLoc = []
        for index in range(0, self.numParticles - 1):
            self.probLoc.append(math.log(1) - math.log(self.numParticles))

        self.sumProbs = 1/self.numParticles

        self.realMean = 0
        self.realStd = 0
        self.particleSense = []
        for index in range(0, self.numParticles-1):
            self.particleSense.append(0)

        self.map = lab10_map.Map("lab10.map")
        #self.virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)

    #single particle first

    def movement(self, x, y, theta):
        self.locationX = x
        self.locationY = y
        self.orientation = theta

    def senseOnce(self, create, sonar):
        self.realStd = .001

        """ distanceArr = []
        for x in range(0, self.numParticles-1):
            distanceArr.append(sonar.get_distance())

        print(distanceArr)
        self.realMean = np.average(distanceArr)
        self.realStd = np.std(distanceArr)
        print("STD", self.realStd)
        #self.probSensorGivenLoc = stats.norm.pdf(self.realMean, self.realStd)
        """

    def sensing(self, create, sonar, virtual_create):
        sensedDistance = sonar.get_distance()
        self.estimation(sensedDistance, virtual_create)

    def estimation(self, sensedDistance, virtual_create):
        #step 1: create 100 randomly distributed particles

        particleX = np.random.uniform(0, 3, self.numParticles)
        particleY = np.random.uniform(0, 3, self.numParticles)
        particleTheta = np.random.uniform(0,2*math.pi, self.numParticles)


        data = []
        for index in range(0, self.numParticles-1):
            data.append(particleX[index])
            data.append(particleY[index])
            data.append(0)
            data.append(particleTheta[index])
        virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)
        virtual_create.set_point_cloud(data)

        #getting sensor readings for each of the random robots

        probabilities_toWeight = []
        virtualProbSensor = []
        for index in range(0, self.numParticles-1):
            virtualProbSensor.append(0)

        sum = []
        for reading in range(0, self.numParticles-1):
            #sensor reading for each virtual robot
            self.particleSense[reading] = self.map.closest_distance((particleX[reading], particleY[reading]), particleTheta[reading])
            #N = 1/self.sumProbs

            #create random normal distribution and get probability
            randDist = np.random.normal(self.realMean, self.realStd)

            virtualProbSensor[reading] = (stats.norm.pdf(sensedDistance, self.particleSense[reading], np.std(randDist)))
            #overall probability of this reading
            sum.append(virtualProbSensor[reading] * self.probLoc[reading])

        totalSum = 0
        for prob in range(0, self.numParticles-1):
            totalSum = sum[prob] + totalSum

        N = 1/totalSum
        for reading in range(0, self.numParticles-1):
            probability = (virtualProbSensor[reading] * (self.probLoc[reading])) * N
            probabilities_toWeight.append(probability)


        #DIVIDE BY ONE ERROR: probably coming from getting NaN or infinite value somewhere up here ^^^

        #now that we have P(virtual robot), kill off useless ones
        resampledRobots = np.random.choice(self.particleSense, self.numParticles, True, probabilities_toWeight)
        copyOfResample = []
        for index in range(0, self.numParticles-1):
            copyOfResample.append(resampledRobots[index])
        print("Resampled: ", resampledRobots)
        self.particleSense = resampledRobots
        #updating probabilities
        self.probLoc = probabilities_toWeight
        self.sumProbs += self.sumProbs
        #if particle is within 1 std deviation of mean value,



