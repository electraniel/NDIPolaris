from enum import IntEnum
class HandleStatus(IntEnum):
    VALID = 1
    MISSING = 2
    DISABLED = 4
    
import numpy as np
import Rotation_Plate as rot
import ndi_polaris_04_07_mac as ndi
import pickle 


p = ndi.Polaris()
plate_roms = ndi.get_plate_roms()
#in setup insert path to rom in []
p.setup(['../roms/P937.rom'])
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
        return rotation, translation, M 




final_position = position_plate()
pr = ndi.get_plate_roms()
plate_rom_lists = [
[pr[1],pr[7],pr[18],pr[32],pr[41],pr[49]], 
[pr[2],pr[5],pr[8],pr[39],pr[42],pr[45]], 
[pr[3],pr[13],pr[16],pr[40],pr[46],pr[52]],    
[pr[9],pr[12],pr[25],pr[26],pr[47],pr[48]],  
[pr[0],pr[10],pr[11],pr[23],pr[24]],
[pr[4],pr[21],pr[33],pr[36],pr[44]],
[pr[6],pr[15],pr[27],pr[30],pr[37]],
[pr[14],pr[17],pr[28],pr[29],pr[51]],
[pr[19],pr[22],pr[35],pr[38],pr[50]],
[pr[20],pr[31],pr[34],pr[43],pr[53]],
    ]
mname = ['A','B','C','D','E','F','G','H','I','J']


def start_plate_measurement(i=0):
    p.setup(plate_rom_lists[i])
    position = [final_position]
    track_data = position + p.perform_tracking( duration = 120., record = lambda *_: True)
    pickle_name = 'Measurement_{}TMR'.format(mname[i])
    with open(pickle_name, 'wb') as handle:
        pickle.dump(track_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
    

for i in range (10):
    start_plate_measurement(i)



#pickle_name = 'Test{},{}.pkl'.format(rom_start,rom_end)


    
"""
M = rot.get_rot_matrix(x = rot.load_measurement('/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/Measuring_Scripts/Plate_test.pkl') )

print (M)
"""