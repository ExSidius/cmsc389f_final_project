import numpy as np

class ActionSpace():
    def __init__(self, size):
        self.size = size
        self.rows = np.power(2, self.size)
        self.actions = np.zeros((self.rows,self.size))
        c = 0
        t = self.rows
        while(c < size):
            t = t/2
            x = t
            i = 0
            while(i < self.rows):
                if(x > 0):
                    self.actions[i,c] = 1
                    x -= 1
                elif(x > -t + 1):
                    self.actions[i,c] = 0
                    x -= 1
                else:
                    x = t
                i += 1
            c += 1

        temp = np.zeros((8, 8))
        temp[np.diag_indices(8, 2)] = 1
        self.actions = np.vstack((np.zeros(8), temp))

    def sample(self):
        t = np.random.randint(self.rows)
        t = np.random.randint(9)
        return t
            