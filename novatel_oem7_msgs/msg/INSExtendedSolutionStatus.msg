
uint32 POSITION_UPDATE              = 1             # 0x00000001
uint32 PHASE_UPDATE                 = 2             # 0x00000002
uint32 ZERO_VELOCITY_UPDATE         = 4             # 0x00000004
uint32 WHEEL_SEONSOR_UPDATE         = 8             # 0x00000008
uint32 ALIGN_UPDATE                 = 16            # 0x00000010
uint32 EXTERNAL_POSITION_UPDATE     = 32            # 0x00000020
uint32 INS_SOLUTIN_CONVERGENCE_FLAG = 64            # 0x00000040
uint32 DOPPLER_UPDATE               = 128           # 0x00000080
uint32 PSEUDORANGE_UPDATE           = 256           # 0x00000100
uint32 VELOCITY_UPDATE              = 512           # 0x00000200
uint32 RESERVED_1                   = 1024          # 0x00000400
uint32 DEAD_RECONING_UPDATE         = 2048          # 0x00000800
uint32 PHASE_WIND_UP_UPDATE         = 4096          # 0x00001000
uint32 COURSE_OVER_GROUND_UPDATE    = 8192          # 0x00002000
uint32 EXTERNAL_VELOCITY_UPDATE     = 16384         # 0x00004000
uint32 EXTERNAL_ATTITUDE_UPDATE     = 32768         # 0x00008000
uint32 EXTERNAL_HEADING_UPDATE      = 65535         # 0x00010000
uint32 EXTERNAL_HEIGHT_UPDATE       = 131072        # 0x00020000
uint32 RESERVED_2                   = 262144        # 0x00040000
uint32 RESERVED_3                   = 524288        # 0x00080000
uint32 ROVER_POSITION_UPDATE        = 1048576       # 0x00100000
uint32 ROVER_POSITION_UPDATE_TYPE   = 2097152       # 0x00200000

uint32 RESERVED_4                   = 4194304       # 0x00400000
uint32 RESERVED_5                   = 8388608       # 0x00800000
uint32 TURN_ON_BIASES_ESTIMATED     = 16777216      # 0x01000000
uint32 ALIGNMENT_DIRECTION_VERIFIED = 33554432      # 0x02000000
uint32 ALIGNMENT_INDICATION_1       = 67108864      # 0x04000000
uint32 ALIGNMENT_INDICATION_2       = 134217728     # 0x08000000
uint32 ALIGNMENT_INDICATION_3       = 268435456     # 0x10000000
uint32 NVM_SEED_INDICATION_1        = 538870912     # 0x20000000
uint32 NVM_SEED_INDICATION_2        = 1073741824    # 0x40000000
uint32 NVM_SEED_INDICATION_3        = 2147483648    # 0x80000000



# Alignment indication
uint32 ALIGNMENT_INCOMPLETE_ALIGNMENT = 0
uint32 ALIGNMENT_STATIC               = 1
uint32 ALIGNMENT_KINETMATIC           = 2
uint32 ALIGNMENT_DUAL_ANTENNA         = 3
uint32 ALIGNMENT_USER_COMMAND         = 4
uint32 ALIGNMENT_NVM_SEED             = 5

# NVM Seed Indication
uint32 NVM_SEED_INACTIVE                          = 0
uint32 NVM_SEED_STORED_INVALID                    = 1
uint32 NVM_SEED_PENDING_VALIDATION                = 2
uint32 NVM_SEED_INJECTED                          = 4
uint32 NVM_SEEED_DATA_IGNORED                     = 5
uint32 NVM_SEED_ERROR_MODEL_DATA_INJECTED         = 6



uint32 status
