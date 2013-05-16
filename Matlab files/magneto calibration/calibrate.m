[magcor_U,magcor_c] = calib_magneto(Mag);
apply()
save('lastcalibrate.mat', 'magcor_U', 'magcor_c');