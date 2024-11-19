rotm_simcam2gencam = ([0  0 1;
                      -1  0 0;
                      0 -1 0]);

rotm_simquad2genquad = ([1  0 0;
                         0  -1 0;
                         0 0 -1]);

instr_in = eul2rotm([pi/2, 0, 0], 'XYZ');

R = transpose((rotm_simcam2gencam)*transpose(instr_in)*transpose(rotm_simquad2genquad));

instr_out = rotm2eul(R, 'XYZ')
