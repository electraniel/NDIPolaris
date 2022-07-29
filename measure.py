from enum import IntEnum
class HandleStatus(IntEnum):
    VALID = 1
    MISSING = 2
    DISABLED = 4
    
import numpy as np
import Rotation_Plate as rot
import ndi_polaris_04_07 as ndi
import pickle 


p = ndi.Polaris()
plate_roms = ndi.get_plate_roms()
#in setup insert path to rom in []
p.setup(['/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/roms/P937.rom'])
p.prepare_tracking()

#Gibt Eulerwinkel und Tranlations unterschied an. 
#mit strg + c wird es abgebrochen und die letzte gespeicherte Position (wingel + Translation wird zurückgegeben)

def position_plate(): 
    try: 
    
        while True:
            track_data = p.perform_tracking( duration = 3., record = lambda *_: True)
            frame = 50
            x = np.array(track_data[frame]['stray_markers']['markers']) # waehlt die Marker aus (aus welchen Frame?)
            x = x[:,2:] # schaut das nur x,y,z da sind
            pickle_name = ('Test6.pkl')
            
            with open(pickle_name, 'wb') as handle:
                pickle.dump(track_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
            M = rot.get_rot_matrix(x)
            X_hom = np.hstack((x, np.ones((x.shape[0], 1))))
            X = M @ X_hom.T
            
            B = rot.load_reference() #ladet die Referenz Platte
            rotation = rot.rot2eul(M[:3,:3])
            translation = M[:3,3]
            print('rotation: ',rotation) # Gibt Eulerwinkel Unterschied an 
            print('Translation',translation)
    
    except KeyboardInterrupt:
        return rotation, translation

def start_plate_measurement():
    p.setup()
    
    pass



final_position = position_plate()
plate_roms = ndi.get_plate_roms()
rom_start, rom_end = 0 , 18
romlist = [plate_roms[rom_start:rom_end]]
#p.setup(*romlist)
p.setup([plate_roms[0],plate_roms[1],plate_roms[5],plate_roms[11],plate_roms[18]])
p.prepare_tracking()
track_data = p.perform_tracking( duration = 100., record = lambda *_: True)
#pickle_name = 'Test{},{}.pkl'.format(rom_start,rom_end)
pickle_name = 'Measurment_0_1_5_11_18.pkl'.format(rom_start,rom_end)
with open(pickle_name, 'wb') as handle:
    pickle.dump(track_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    
"""
M = rot.get_rot_matrix(x = rot.load_measurement('/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/Measuring_Scripts/Plate_test.pkl') )

print (M)
"""