uint64 timestamp

# controller output
float32[3] u

# controller states
float32[3] ueq
float32[3] k
float32[3] r

float32[3] ut

# controller input
float32[3] sigma
float32[3] derr
float32[3] delta
float32[3] rho

# vehicle attitude
float32[3] att
float32[3] att_sp
