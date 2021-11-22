from sympy import symbols, Matrix, cos, sin


# Symbols
x_utm, y_utm, z_utm = symbols('x_utm, y_utm, z_utm', real=True)
r_imu, p_imu, y_imu = symbols('r_imu, p_imu, y_imu', real=True)
r_bore, p_bore, y_bore = symbols('r_bore, p_bore, y_bore', real=True)
sa_rl, sa_fb, d = symbols('sa_rl, sa_fb, d', real=True)
x_lever, y_lever, z_lever = symbols('x_lever, y_lever, z_lever', real=True)


# IMU Frame: X forward, Y to the right, Z down
# These are CCW positive, active matrices
R1_imu = Matrix([[1, 0, 0],
                 [0, cos(r_imu), -sin(r_imu)],
                 [0, sin(r_imu), cos(r_imu)]])

R2_imu = Matrix([[cos(p_imu), 0, sin(p_imu)],
                 [0, 1, 0],
                 [-sin(p_imu), 0, cos(p_imu)]])

R3_imu = Matrix([[cos(y_imu), -sin(y_imu), 0],
                 [sin(y_imu), cos(y_imu), 0],
                 [0, 0, 1]])

R_imu = R3_imu * R2_imu * R1_imu
# # Temp for PDAL filter inversion code (R1 not included since we do not estimate roll)
# R_temp = R3_imu * R2_imu
# print(R_temp.T)
# exit()


# Boresight rotations will use the same angle names and rotation order as IMU
# These are CCW positive, passive matrices
R1_bore = Matrix([[1, 0, 0],
                 [0, cos(r_bore), -sin(r_bore)],
                 [0, sin(r_bore), cos(r_bore)]])

R2_bore = Matrix([[cos(p_bore), 0, sin(p_bore)],
                 [0, 1, 0],
                 [-sin(p_bore), 0, cos(p_bore)]])

R3_bore = Matrix([[cos(y_bore), -sin(y_bore), 0],
                 [sin(y_bore), cos(y_bore), 0],
                 [0, 0, 1]])

R_bore = R3_bore * R2_bore * R1_bore


# Scanner frame is parallel to IMU frame: X forward, Y to the right, Z down
# These are CCW positive, active matrices
R1 = Matrix([[1, 0, 0],
             [0, cos(-sa_rl), -sin(-sa_rl)],
             [0, sin(-sa_rl), cos(-sa_rl)]])

R2 = Matrix([[cos(sa_fb), 0, sin(sa_fb)],
             [0, 1, 0],
             [-sin(sa_fb), 0, cos(sa_fb)]])

d_nadir = Matrix([[0],
                  [0],
                  [d]])

R = R1 * R2
xyz_laser = R * d_nadir

# Change basis from NED to ENU ("local level") to align with UTM/SPC/etc.
R_cob = Matrix([[0, 1, 0],
                [1, 0, 0],
                [0, 0, -1]])

# Sensor location and lever arm vectors
xyz_utm = Matrix([[x_utm],
                  [y_utm],
                  [z_utm]])
xyz_lever = Matrix([[x_lever],
                    [y_lever],
                    [z_lever]])


# Full equation
xyz_ground = xyz_utm + R_cob*R_imu*(R_bore*xyz_laser + xyz_lever)
x = xyz_ground[0]
y = xyz_ground[1]
z = xyz_ground[2]
diff_vars = (d, sa_rl, sa_fb, x_utm, y_utm, z_utm, r_imu, p_imu, y_imu, r_bore, p_bore, y_bore, x_lever, y_lever, z_lever)
for var in diff_vars:
    deriv = x.diff(var)
    print('x diff {} = {}'.format(var, deriv))
for var in diff_vars:
    deriv = y.diff(var)
    print('y diff {} = {}'.format(var, deriv))
for var in diff_vars:
    deriv = z.diff(var)
    print('z diff {} = {}'.format(var, deriv))

# # No lever
# xyz_ground = xyz_utm + R_imu*(R_bore*xyz_laser)

# # No bore
# xyz_ground = xyz_utm + R_imu*(xyz_laser + xyz_lever)

# No lever, no bore
# xyz_ground = xyz_utm + R_imu*xyz_laser
# x = xyz_ground[0]
# y = xyz_ground[1]
# z = xyz_ground[2]
# diff_vars = (d, sa_rl, sa_fb, x_utm, y_utm, z_utm, r_imu, p_imu, y_imu)
# for var in diff_vars:
#     deriv = x.diff(var)
#     print('x diff {} = {}'.format(var, deriv))
# for var in diff_vars:
#     deriv = y.diff(var)
#     print('y diff {} = {}'.format(var, deriv))
# for var in diff_vars:
#     deriv = z.diff(var)
#     print('z diff {} = {}'.format(var, deriv))