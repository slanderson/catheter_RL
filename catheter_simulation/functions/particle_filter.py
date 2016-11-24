import numpy as np
import math, random, collections




# Function: Pdf
# -------------------------
# Returns the Gaussian probability density of a distribution with
# a given mean and std producing a given value.
def pdf(mean, std, value):
    u = float(value - mean) / abs(std)
    y = (1.0 / (math.sqrt(2 * math.pi) * abs(std))) * math.exp(-u * u / 2.0)
    return y

# Function: Weighted Random Choice
# --------------------------------
# Given a dictionary of the form element -> weight, selects an element
# randomly based on distribution proportional to the weights. Weights can sum
# up to be more than 1. 
def weightedRandomChoice(weightDict):
    weights = []
    elems = []
    for elem in weightDict:
        weights.append(weightDict[elem])
        elems.append(elem)
    total = sum(weights)
    # print(total)
    key = random.uniform(0, total)
    runningTotal = 0.0
    chosenIndex = None
    for i in range(len(weights)):
        weight = weights[i]
        runningTotal += weight
        if runningTotal > key:
            chosenIndex = i
            return elems[chosenIndex]
    raise Exception('Should not reach here')

# Class: Particle Filter
# ----------------------
# Maintain and update a belief distribution over the probability of a car
# being in a tile using a set of particles.
class ParticleFilter(object):
    
    NUM_PARTICLES = 20
    # Sensor_STD = 1
    # Move_STD = 0.01
    Sensor_STD = 1
    Move_STD = 0.01
    
    # Function: Init
    # --------------
    # Constructer that initializes an ParticleFilter object
    def __init__(self, x_initial):
        # Initializes the particle filter around the initial position. 
        # self.transProbDict = dict()
        # for (oldTile, newTile) in self.transProb:
        #     if not oldTile in self.transProbDict:
        #         self.transProbDict[oldTile] = collections.Counter()
        #     self.transProbDict[oldTile][newTile] = self.transProb[(oldTile, newTile)]
            
        # Initialize the particles randomly around the initial position.  
        self.particles = collections.Counter()
        # potentialParticles = self.transProbDict.keys()
        for i in range(self.NUM_PARTICLES):
            particle_coords = [x + random.gauss(0, self.Sensor_STD) for x in x_initial]
            self.particles[tuple(particle_coords)] += 1. / self.NUM_PARTICLES
        # print(self.particles)
        # not yet sure how the belief object is used or if it's necessary here
        # self.updateBelief()

    # Function: Update Belief
    # ---------------------
    # Updates |self.belief| with the probability that the car is in each tile
    # based on |self.particles|, which is a Counter from particle to
    # probability (which should sum to 1).
    def updateBelief(self):
        newBelief = util.Belief(self.belief.getNumRows(), self.belief.getNumCols(), 0)
        for tile in self.particles:
            newBelief.setProb(tile[0], tile[1], self.particles[tile])
        newBelief.normalize()
        self.belief = newBelief
    
    ############################################################
    # Takes |self.particles| and updates them based on the sensor observation
    # This algorithm takes two steps:
    # 1. Reweight the particles based on the observation.
    #    Concept: We had an old distribution of particles, we want to update these
    #             these particle distributions with the given observed distance by
    #             the emission probability. 
    #             Think of the particle distribution as the unnormalized posterior 
    #             probability where many tiles would have 0 probability.
    #             Tiles with 0 probabilities (no particles), we do not need to update. 
    #             This makes particle filtering runtime to be O(|particles|).
    #             In comparison, exact inference (problem 2 + 3), most tiles would
    #             would have non-zero probabilities (though can be very small). 
    # 2. Resample the particles.
    #    Concept: Now we have the reweighted (unnormalized) distribution, we can now 
    #             resample the particles and update where each particle should be at.

    ############################################################
    def observe(self, x_sensor):
        # BEGIN_YOUR_CODE (around 15 lines of code expected)
        def Norm2(x_array):
            return math.sqrt(sum([x**2 for x in x_array]))

        for x_particle in self.particles:
            this_dist = Norm2(np.asarray(x_particle) - x_sensor)
            this_pdf = pdf(0, self.Sensor_STD, this_dist) # pdf around the sensor location
            self.particles[x_particle] *= this_pdf

        # self.particles[tuple(x_sensor)] += 1. / self.NUM_PARTICLES
        # print(self.particles)
        # weights = []
        # for elem in self.particles:
        #     weights.append(self.particles[elem])
        # total = sum(weights)
        # if total == 0:
        #     print('redo')
        #     new_particles = collections.Counter()
        #     for i in range(self.NUM_PARTICLES):
        #         particle_coords = [x + random.gauss(0, self.Sensor_STD) for x in x_sensor]
        #         new_particles[tuple(particle_coords)] += 1. / self.NUM_PARTICLES
        # else:
        new_particles = collections.Counter()
        for i in range(self.NUM_PARTICLES):
            new_particles[weightedRandomChoice(self.particles)] += 1. / self.NUM_PARTICLES

        self.particles = new_particles
        # END_YOUR_CODE
        # self.updateBelief()
    
    ############################################################
    # Problem 4 (part b): 
    # Function: Elapse Time (propose a new belief distribution based on a learned transition model)
    # ---------------------
    # Read |self.particles| (Counter) corresonding to time $t$ and writes
    # |self.particles| corresponding to time $t+1$.
    # This algorithm takes one step
    # 1. Proposal based on the particle distribution at current time $t$:
    #    Concept: We have particle distribution at current time $t$, we want to
    #             propose the particle distribution at time $t+1$. We would like
    #             to sample again to see where each particle would end up using
    #             the transition model.
    #
    # Notes:
    # - transition probabilities is now using |self.transProbDict|
    # - Use util.weightedRandomChoice() to sample a new particle.
    # - To pass the grader, you must loop over the particles using
    #       for tile in self.particles
    #   and call util.weightedRandomChoice() $once per particle$ on the tile.
    ############################################################
    def elapseTime(self, dx_desired):
        # BEGIN_YOUR_CODE (around 10 lines of code expected)
        new_particles = collections.Counter()
        for x_particle in self.particles:
            moved_particle = [x + random.gauss(0.1*(dx_desired[i] - x), self.Move_STD) for i, x in enumerate(x_particle)]
            # moved_particle = x_particle
            new_particles[tuple(moved_particle)] += 1. / self.NUM_PARTICLES
        self.particles = new_particles
        # print(self.particles)
        # END_YOUR_CODE
        
    # Function: Get Belief
    # ---------------------
    # Returns your belief of the probability that the car is in each tile. Your
    # belief probabilities should sum to 1.    
    def getBelief(self):
        return self.belief