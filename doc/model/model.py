from sympy import symbols, Matrix, cos, sin


# Symbols
trajX, trajY, trajZ = symbols('trajX, trajY, trajZ', real=True)
trajRoll, trajPitch, trajHeading = symbols('trajRoll, trajPitch, trajHeading', real=True)
boreRoll, borePitch, boreYaw = symbols('boreRoll, borePitch, boreYaw', real=True)
scanAngleRL, scanAngleFB, lidarDist = symbols('scanAngleRL, scanAngleFB, lidarDist', real=True)
leverX, leverY, leverZ = symbols('leverX, leverY, leverZ', real=True)


# IMU Frame: X forward, Y to the right, Z down
# Using counter-clockwise positive, active rotation matrices
Rx_imu = Matrix([[1, 0, 0],
                 [0, cos(trajRoll), -sin(trajRoll)],
                 [0, sin(trajRoll), cos(trajRoll)]])

Ry_imu = Matrix([[cos(trajPitch), 0, sin(trajPitch)],
                 [0, 1, 0],
                 [-sin(trajPitch), 0, cos(trajPitch)]])

Rz_imu = Matrix([[cos(trajHeading), -sin(trajHeading), 0],
                 [sin(trajHeading), cos(trajHeading), 0],
                 [0, 0, 1]])

R_imu = Rz_imu * Ry_imu * Rx_imu

# # Output for PDAL filter inversion code (Rx not included since we do not
# consider roll (it is not estimated in filters.sritrajectory)
# R_invert = Rz_imu * Ry_imu
# print(R_invert.T)
# exit()


# Boresight rotations are defined the same as the IMU rotations
Rx_bore = Matrix([[1, 0, 0],
                  [0, cos(boreRoll), -sin(boreRoll)],
                  [0, sin(boreRoll), cos(boreRoll)]])

Ry_bore = Matrix([[cos(borePitch), 0, sin(borePitch)],
                  [0, 1, 0],
                  [-sin(borePitch), 0, cos(borePitch)]])

Rz_bore = Matrix([[cos(boreYaw), -sin(boreYaw), 0],
                  [sin(boreYaw), cos(boreYaw), 0],
                  [0, 0, 1]])

R_bore = Rz_bore * Ry_bore * Rx_bore


# Scanner frame is parallel to IMU frame: X forward, Y to the right, Z down
# Using counter-clockwise positive, active rotation matrices
Rx_scanner = Matrix([[1, 0, 0],
                     [0, cos(-scanAngleRL), -sin(-scanAngleRL)],
                     [0, sin(-scanAngleRL), cos(-scanAngleRL)]])

Ry_scanner = Matrix([[cos(scanAngleFB), 0, sin(scanAngleFB)],
                     [0, 1, 0],
                     [-sin(scanAngleFB), 0, cos(scanAngleFB)]])

d_nadir = Matrix([[0],
                  [0],
                  [lidarDist]])

R_scanner = Rx_scanner * Ry_scanner
xyz_laser = R_scanner * d_nadir

# Change Local Level basis from NED to ENU to align with projected coordinate
# system
R_cob = Matrix([[0, 1, 0],
                [1, 0, 0],
                [0, 0, -1]])

# Sensor location and Lsummever arm vectors
xyz_sensor = Matrix([[trajX],
                     [trajY],
                     [trajZ]])
xyz_lever = Matrix([[leverX],
                    [leverY],
                    [leverZ]])


# Full equation
xyz_ground = xyz_sensor + R_cob*R_imu*(R_bore*xyz_laser + xyz_lever)
x = xyz_ground[0]
y = xyz_ground[1]
z = xyz_ground[2]
print('x = {}\n'.format(x))
print('y = {}\n'.format(y))
print('z = {}\n\n'.format(z))

# Partial derivatives
diff_vars = (lidarDist, scanAngleRL, scanAngleFB, trajX, trajY, trajZ, trajRoll, trajPitch, trajHeading, boreRoll, borePitch, boreYaw, leverX, leverY, leverZ)
for var in diff_vars:
    deriv = x.diff(var)
    print('x diff {} = {}'.format(var, deriv))
for var in diff_vars:
    deriv = y.diff(var)
    print('y diff {} = {}'.format(var, deriv))
for var in diff_vars:
    deriv = z.diff(var)
    print('z diff {} = {}'.format(var, deriv))
