# Sample rate 1 Hz for setup
!UBX CFG-RATE 1000 1 1

# Configure GPS and GLONASS satellites
!UBX CFG-GNSS 0 32 32 1 0 10 32 0 65537
!UBX CFG-GNSS 0 32 32 1 6 8 16 0 65537

# Set up raw data out for UART and USB
!UBX CFG-MSG 3 15 0 1 0 1 0 0
!UBX CFG-MSG 3 16 0 1 0 1 0 0
!UBX CFG-MSG 1 32 0 1 0 1 0 0

# change NAV5 mode to pedestrian
!UBX CFG-NAV5 1 3 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

# turn off extra messages default messages
# NMEA GGA
!UBX CFG-MSG 240 0 0 0 0 0 0 0
# NMEA GLL
!UBX CFG-MSG 240 1 0 0 0 0 0 0
# NMEA GSA
!UBX CFG-MSG 240 2 0 0 0 0 0 0
# NMEA GSV
!UBX CFG-MSG 240 3 0 0 0 0 0 0
# NMEA RMC
!UBX CFG-MSG 240 4 0 0 0 0 0 0
# NMEA VTG
!UBX CFG-MSG 240 5 0 0 0 0 0 0
# NMEA ZDA + others
!UBX CFG-MSG 240 8 0 0 0 0 0 0
!UBX CFG-MSG 1 3 0 0 0 0 0 0
!UBX CFG-MSG 1 3 0 0 0 0 0 0
!UBX CFG-MSG 1 6 0 0 0 0 0 0
!UBX CFG-MSG 1 18 0 0 0 0 0 0
!UBX CFG-MSG 1 34 0 0 0 0 0 0
!UBX CFG-MSG 1 48 0 0 0 0 0 0

# Sample rate 5 Hz for data collection
!UBX CFG-RATE 200 1 1




@
!UBX CFG-RATE 1000 1 1