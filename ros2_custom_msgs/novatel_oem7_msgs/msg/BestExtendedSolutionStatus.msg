# Bit 0
uint8 RTK_SOLUTION_VERIFIED = 1
uint8 PDP_SOLUTION_IS_GLIDE = 1

# Bits 1-3, mask 0xE
uint8 KLOBUCHAR_BROADCAST                =  2 # 0x2 
uint8 SBAS_BROADCAST                     =  4 # 0x4 
uint8 MULTI_FREQUENCY_COMPUTED           =  6 # 0x6
uint8 PSRDIFF_CORRECTION                 =  8 # 0x8
uint8 NOVATEL_BLENDED_IONO_VALUE         = 10 #0xA

# Bit 4
uint8 RTK_ASSIST_ACTIVE = 16 # 0x10

# Bit 5
uint8 ANTENNA_INFORMATION_IS_MISSING = 32 # 0x20

# Bit 6 reserved
uint8 RESERVED = 64 # 0x40

# Bit 7
uint8 POSITION_INCLUDES_TERRAIN_COMPENSATION_CORRECTIONS = 128 # 0x80



uint8 status