eul_imu2rq = [0 0 -90];
eul_rw2rq = [(180) 0 0];
w = [0 0 0 0];
a = [12; 2; 10];
g = [0; 0; -9.81];


q_imu2rq = eul2quat(deg2rad(eul_imu2rq), 'XYZ');
q_rw2rq = eul2quat(deg2rad(eul_rw2rq), 'XYZ');
q = q_rw2rq;

lmo_imu2rq =  [q_imu2rq(1) -q_imu2rq(2) -q_imu2rq(3) -q_imu2rq(4)
                q_imu2rq(2) q_imu2rq(1) -q_imu2rq(4) q_imu2rq(3)
                q_imu2rq(3) q_imu2rq(4) q_imu2rq(1) -q_imu2rq(2)
                q_imu2rq(4) -q_imu2rq(3) q_imu2rq(2) q_imu2rq(1)];

q_rq2rw = [q(1) -q(2) -q(3) -q(4)];
lmo_rq2rw = [q_rq2rw(1) -q_rq2rw(2) -q_rq2rw(3) -q_rq2rw(4)
            q_rq2rw(2) q_rq2rw(1) -q_rq2rw(4) q_rq2rw(3)
            q_rq2rw(3) q_rq2rw(4) q_rq2rw(1) -q_rq2rw(2)
            q_rq2rw(4) -q_rq2rw(3) q_rq2rw(2) q_rq2rw(1)];

R_rw2rq = [1 - 2*(q(3)^2 + q(4)^2), 2*(q(2)*q(3) - q(4)*q(1)), 2*(q(2)*q(4) + q(3)*q(1));
    2*(q(2)*q(3) + q(4)*q(1)), 1 - 2*(q(2)^2 + q(4)^2), 2*(q(3)*q(4) - q(2)*q(1));
    2*(q(2)*q(4) - q(3)*q(1)), 2*(q(3)*q(4) + q(2)*q(1)), 1 - 2*(q(2)^2 + q(3)^2)];

a1 = ((transpose(R_rw2rq) * R_imu2rq) * (- a) + g)

a2 =  [-1 0 0; 0 -1 0; 0 0 1] * ((transpose(R_rw2rq) * R_imu2rq) *( -a) + g)