import ndi_polaris_04_07 as ndi
import Rotation_Plate as rot
import pickle

p = ndi.Polaris()
ndi.get_plate_roms()
p.setup([ndi.plate_roms[1],ndi.plate_roms[5]])
#p.setup()
p.prepare_tracking()
track_data = p.perform_tracking()
pickle_name = "Measurement_Plate_1_5"
with open(pickle_name, 'wb') as handle:
    pickle.dump(track_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
    
    
