GyroSD = std(Gyro)*180/(818.51113590117601252569*pi);
% 2000dps range: 818.51113590117601252569
% 500dps range: 3274.04454360470405010275
% 250dps range: 6548.0890872094081002055
fprintf(1, 'Gyro: ');
fprintf(1, '%f  ', GyroSD);
fprintf(1, 'dps\n');

AccelSD = std(Accel)/(2^14);
% +/-2g range: 2^14
% +/-4g range: 2^11
% +/-8g range: 2^10
% +/-16g range: 2^9
fprintf(1, 'Acce: ');
fprintf(1, '%f  ', AccelSD);
fprintf(1, 'g\n');

AccelSD = std(Accel)/(2^14)*180/pi;
% +/-2g range: 2^14
% +/-4g range: 2^11
% +/-8g range: 2^10
% +/-16g range: 2^9
fprintf(1, 'Acce: ');
fprintf(1, '%f  ', AccelSD);
fprintf(1, 'equiv deg\n');

MagSD = std(Mag)/1090;
% +/-0.88Ga range: 1370
% +/-0.1.3Ga range: 1090
% +/-0.1.9Ga range: 820
% ...
fprintf(1, 'Magn: ');
fprintf(1, '%f  ', MagSD);
fprintf(1, 'Ga\n');

MagSD = std(Mag)/1090*180/pi;
% +/-0.88Ga range: 1370
% +/-0.1.3Ga range: 1090
% +/-0.1.9Ga range: 820
% ...
fprintf(1, 'Magn: ');
fprintf(1, '%f  ', MagSD);
fprintf(1, 'equiv deg\n');

clear GyroSD AccelSD MagSD;