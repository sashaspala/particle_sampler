import numpy as np
from scipy import stats, misc
import time
import lab10_map
import math
from collections import namedtuple, Counter
Particle = namedtuple("Particle", "index x y z theta")

class ParticleFilter:

    def __init__(self):
        self.time = time
        self.locationX = 0
        self.locationY = 0
        self.orientation = 0

        self.distance = 0

        #self.probSensorGivenLoc = 0
        self.numParticles = 1000
        self.probLoc = []
        for index in range(0, self.numParticles):
            #self.probLoc.append(math.log(1) - math.log(self.numParticles))
            self.probLoc.append(1/self.numParticles)

        self.realMean = 0
        self.realStd = 0

        self.particleSense = []
        for index in range(0, self.numParticles):
            self.particleSense.append(0)

        self.round = 1
        self.data = []
        self.particlesToUse = []
        self.map = lab10_map.Map("lab10.map")
        #self.virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)

    #single particle first

    def movement(self, x, y, theta):
        self.locationX = x
        self.locationY = y
        self.orientation = theta

    def senseOnce(self, create, sonar):
        self.realStd = .05

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
        if(self.round == 1):

            particleX = np.random.uniform(0, 3, self.numParticles)
            particleY = np.random.uniform(0, 3, self.numParticles)
            particleTheta = np.random.uniform(0,2*math.pi, self.numParticles)


            #data = []
            for index in range(0, self.numParticles):
                self.data.append(particleX[index])
                self.data.append(particleY[index])
                self.data.append(0)
                self.data.append(particleTheta[index])

            #virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)
            virtual_create.set_point_cloud(self.data)

            # now add these values to the namedtuple to use later
            for index in range(0, self.numParticles):
                newParticle = Particle(index, particleX[index], particleY[index], 0, particleTheta[index])
                self.particlesToUse.append(newParticle)

            self.round += self.round
        else:
            index = 0
            while index < len(self.data):
                particleIndex = int(index/4)
                self.data[index] = self.particlesToUse[particleIndex].x
                self.data[index + 1] = self.particlesToUse[particleIndex].y
                self.data[index  + 2] = self.particlesToUse[particleIndex].z
                self.data[index + 3] = self.particlesToUse[particleIndex].theta
                index = index + 4


            virtual_create.set_point_cloud(self.data)
        #getting sensor readings for each of the random robots

        probabilities_toWeight = []
        virtualProbSensor = []
        for index in range(0, self.numParticles):
            virtualProbSensor.append(0)

        sum = []
        reading = 0

        for reading in range(0, self.numParticles):
            #sensor reading for each virtual robot
            self.particleSense[reading] = self.map.closest_distance((self.particlesToUse[reading].x, self.particlesToUse[reading].y), self.particlesToUse[reading].theta)

            #create random normal distribution and get probability
            randDist = np.random.normal(np.average(self.particleSense[reading]), self.realStd)
            virtualProbSensor[reading] = (stats.norm.pdf(sensedDistance, self.particleSense[reading], self.realStd))

            #overall probability of this reading
            sum.append(virtualProbSensor[reading] * self.probLoc[reading])


        totalSum = 0
        for prob in range(0, self.numParticles):
            totalSum = sum[prob] + totalSum

        N = 1/totalSum

        arrayIndex = []
        for reading in range(0, self.numParticles):
            probability = (virtualProbSensor[reading] * (self.probLoc[reading])) * N
            probabilities_toWeight.append(probability)

            #cast list to array at same time
            arrayIndex.append(self.particlesToUse[reading].index)


        #now that we have P(virtual robot), kill off useless ones
        resampledRobots = np.random.choice(arrayIndex, self.numParticles, True, probabilities_toWeight)

        #copy over new values
        copyOfParticles = []
        count = Counter(arrayIndex)
        mostLikely = count.most_common(1)
        if mostLikely[0][1] >= (self.numParticles/3 *2): #if we assume at least 2/3 of the particles makes a good assumption
            find = False
        else:
            find = True

        found = False
        for index in range(0, self.numParticles):
           for oldIndex in range(0, self.numParticles):
               if(resampledRobots[index] == self.particlesToUse[oldIndex].index):
                   copyOfParticles.append(self.particlesToUse[oldIndex])
                   if(resampledRobots[index] == mostLikely[0][0] and ~find):
                        guess = self.particlesToUse[oldIndex]
                        found = True

        if(found):
            virtual_create.set_pose((guess.x, guess.y, guess.z), guess.theta)

        self.particlesToUse = copyOfParticles
        #updating probabilities
        for index in range(0, len(self.probLoc)):
                self.probLoc[index] = probabilities_toWeight[index]
        #if particle is within 1 std deviation of mean value,







