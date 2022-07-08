from enum import IntEnum
class HandleStatus(IntEnum):
    VALID = 1
    MISSING = 2
    DISABLED = 4
    
import numpy as np
import Rotation_Plate_edit as rot
import ndi_polaris_04_07 as ndi
import Rotation_Plate_edit as rot
import pickle 


p = ndi.Polaris()
ndi.get_plate_roms()
p.setup()
p.prepare_tracking()

try: 

    while True:
        track_data = p.perform_tracking( duration = 3., record = lambda *_: True)
        frame = 50
        x = np.array(track_data[frame]['stray_markers']['markers']) # waehlt die Marker aus (aus welchen Frame?)
        x = x[:,2:] # schaut das nur x,y,z da sind
        pickle_name = ('Test3.pkl')
        
        with open(pickle_name, 'wb') as handle:
            pickle.dump(track_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        
        M = rot.get_rot_matrix(x)
        X_hom = np.hstack((x, np.ones((x.shape[0], 1))))
        X = M @ X_hom.T
        
        B = rot.load_reference() #ladet die Referenz Platte
        print(rot.rot2eul(M[:3,:3])) # Gibt Eulerwinkel Unterschied an 
        #rot.plot_comp(B.T,X[:3],"Vergleich") 

except KeyboardInterrupt:
    pass


"""
    M = rot.get_rot_matrix(x = rot.load_measurement('/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/Measuring_Scripts/Plate_test.pkl') )

print (M)
"""