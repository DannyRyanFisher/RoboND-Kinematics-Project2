### Script designed to generate a pickle file from forward kinematics object FK
### in Forward_Kinematics.py

from time import time
import pickle

start_time = time()

from Forward_Kinematics import FK

# Pickle FK
pickle_out = open("FK.pickle", "wb")
pickle.dump(FK, pickle_out)
pickle_out.close()

# Unpickle FK
pickle_in = open("FK.pickle", "rb")
FK = pickle.load(pickle_in)

### Test unpickling a file
#print(FK.T0_1)

### Duration calculation
duration = time()-start_time

#print("Pickling time",duration)


###########################################################
### Inverse Pickling

from Inverse_Kinematics import IK

# Pickle IK
pickle_out = open("IK.pickle", "wb")
pickle.dump(IK, pickle_out)
pickle_out.close()

# Unpickle FK
pickle_in = open("IK.pickle", "rb")
IK = pickle.load(pickle_in)

#print("This is theta4 from IK object (from pickled file)",IK.theta4)

print("WC", IK.WC)
